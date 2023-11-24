#include "Config_Pins.h"
#include "Config_Secrets.h"
#include "Config_Preferences.h"
#include "Config_Actions.h"

#include <EspMQTTClient.h>
#include <Adafruit_NeoPixel.h>
#include <driver/rtc_io.h>
#include <vector>
#include "Keypad.h"
#include "Color.h"
#include "Logger.h"

// ---------------------------
// WiFi/MQTT connection
// ---------------------------
EspMQTTClient mqttClient(WIFI_SSID,
                         WIFI_PASS,
                         MQTT_BROKER_URL,
                         IO_USERNAME,
                         IO_KEY,
                         MQTT_DEVICE_NAME);

enum class ConnectionState {
    uninitialized,
    connecting,
    connected,
    disconnected
};

ConnectionState connectionState = ConnectionState::uninitialized;
uint32_t connectionTimer = 0;

// ---------------------------
// Toggle state tracking
// ---------------------------
struct ToggleState {

    ToggleState(const ToggleAction& aConfig) :
        config(aConfig)
    {
    }

    const ToggleAction config;
    bool toggleState = false;
    bool isBusy = false;
};

typedef std::vector<ToggleState> ToggleStates;
ToggleStates toggleStates;

// ---------------------------
// Status LED
// ---------------------------
Adafruit_NeoPixel statusLED(1, PIN_STATUS_LED, NEO_GRB + NEO_KHZ800);

uint8_t statusLEDBrightness = STATUS_LED_NORMAL_BRIGHTNESS;

// ---------------------------
// Tilt ball switch
// ---------------------------
bool lastTiltState = LOW;
bool tiltState = LOW;

// ------------------------------
// Voltage Monitor
// ------------------------------
uint16_t rawVoltage = 0;
bool isConnectedToUSB = false;

// ------------------------------
// User inactivity timer
// ------------------------------
uint32_t timeSinceUserActivity = 0;

// ---------------------------
// Activity blip
// ---------------------------
const int activityFlashDuration = 150;
int activityFlashTimer = 0;

// ------------------------------
// Boot count tracking
// ------------------------------
// This is really only used for debugging deep sleep,
// though it does provide an example of tracking state
// across sleep sessions.
RTC_DATA_ATTR int bootCount = 0;

// ------------------------------
// Loop time tracking
// ------------------------------
uint32_t lastMillis = 0;

// ---------------------------
// Setup
// ---------------------------
void setup() {
    Serial.begin(115200);

    // Turn off the on-board NeoPixel to save a little bit of power.
    // It's in a case so we can't see it anyway.
    pinMode(NEOPIXEL_POWER, OUTPUT);
    digitalWrite(NEOPIXEL_POWER, (NEOPIXEL_POWER_ON == HIGH) ? LOW : HIGH);

    // Status LED
    initStatusLED();

    // Deep sleep/wake debugging.
    #if defined(LOGGER)
        delay(2000);
        LOGFMT("Boot count: %d\n", ++bootCount);
        logWakeReason();
    #endif        

    // Tilt switch
    initTiltSwitch();

    // Voltage monitor
    initVoltageMonitor();

    // Keypad
    keypad.begin();

    // Create toggle states for each item in the toggles configuration
    initToggleStates();

    // Connect to WiFi
    LOGLN("\nConnecting to WiFi");
    setConnectionState(ConnectionState::connecting);    

    // reset timer
    lastMillis = millis();
}

// ---------------------------
// Loop
// ---------------------------
void loop() {
    uint32_t now = millis();
    uint32_t dt = now - lastMillis;
    lastMillis = now;

    mqttClient.loop();
    updateConnectionState(dt);
    readVoltage();
    readTiltSwitch();
    readKeypad();
    updateStatusLED();

    delay(LOOP_DELAY);
}

// ------------------------------
// WiFi/MQTT Connection handling
// ------------------------------
bool isConnected() {
    return connectionState == ConnectionState::connected;
}

void setConnectionState(ConnectionState state) {
    connectionState = state;

    switch (state) {
        case ConnectionState::connecting:
        case ConnectionState::disconnected:
            connectionTimer = 0;
            break;

        default:
            break;
    }
}

void updateConnectionState(uint32_t dt) {
    switch (connectionState) {
        case ConnectionState::connecting:
            if (mqttClient.isConnected()) {
                setConnectionState(ConnectionState::connected);
            } else {
                connectionTimer += dt;

                if (connectionTimer >= CONNECT_TIMEOUT) {
                    LOGLN("Failed to connect to WiFi before time out, disconnecting");
                    setConnectionState(ConnectionState::disconnected);
                }
            }
            break;

        case ConnectionState::connected:
            if (mqttClient.isConnected()) {
                timeSinceUserActivity = min(timeSinceUserActivity + dt, USER_INACTIVITY_TIMEOUT);
                activityFlashTimer = max(0, activityFlashTimer - (int32_t)dt);

                if (timeSinceUserActivity >= USER_INACTIVITY_TIMEOUT) {
                    deepSleep();
                }                
            } else {
                LOGLN("Lost connection to WiFi or MQTT broker");
                setConnectionState(ConnectionState::disconnected);                
            }
            break;

        case ConnectionState::disconnected:
            if (mqttClient.isConnected()) {
                setConnectionState(ConnectionState::connected);
            } else {
                connectionTimer += dt;

                if (connectionTimer >= RECONNECT_TIMEOUT) {
                    deepSleep();
                }
            }
            break;

        default:
            break;
    }
}

void onConnectionEstablished() {
    LOGLN("Connected to MQTT broker");

    // Subscribe to toggle topics.
    for (ToggleStates::iterator it = toggleStates.begin(); it != toggleStates.end(); ++it) {
        subscribeToggle(*it);
    }
}

void subscribeToggle(ToggleState& toggleState) {
    mqttClient.subscribe(toggleState.config.getToggleTopic(), [&toggleState] (const String &topic, const String& payload) mutable {
        LOGFMT("Received message for topic '%s': %s\n", topic.c_str(), payload.c_str());

        if (payload == "0") {
            toggleState.toggleState = false;
        } else {
            toggleState.toggleState = true;
        }

        toggleState.isBusy = false;
        
        userDidInteract();
        activityFlash();
    });

    // https://io.adafruit.com/api/docs/mqtt.html#using-the-get-topic
    mqttClient.publish(toggleState.config.getToggleTopic() + "/get", "\0");

    toggleState.isBusy = true;
}

bool isAnyToggleBusy() {
    for (ToggleStates::iterator statesIter = toggleStates.begin(); statesIter != toggleStates.end(); ++statesIter) {
        ToggleState& toggleState = *statesIter;

        if (toggleState.isBusy) {
            return true;
        }
    }

    return false;
}

// ------------------------------
// Input
// ------------------------------
void initTiltSwitch() {
    pinMode(PIN_TILT, INPUT_PULLUP);
    tiltState = lastTiltState = digitalRead(PIN_TILT);    
}

void readTiltSwitch() {
    tiltState = digitalRead(PIN_TILT);

    if (tiltState != lastTiltState) {
        if (isConnected()) {
            userDidInteract();
        }
    }

    lastTiltState = tiltState;    
}

void initToggleStates() {
    for (int c = 0; c < toggleConfigCount; c++) {
        const ToggleAction& config = toggleConfigs[c];
        toggleStates.push_back(ToggleState(config));
    }
}

void readKeypad() {
    keypad.tick();

    // Flush and ignore any keypad events if we cannot
    // handle them due to lack of connectivity,
    // or if we're already waiting for a feed update.
    if (!isConnected() || isAnyToggleBusy()) {
        while(keypad.available()) {
            keypad.read();
        }

        return;
    }

    // Process keypad events.
    bool didHandleEvent = false;

    while(keypad.available()) {
        keypadEvent e = keypad.read();
        uint8_t event = e.bit.EVENT;
        char key = (char)e.bit.KEY;

        // Once we've handled an event, we're going to flush and ignore the rest
        // of the events remaining in the buffer. These matrix keypads aren't designed 
        // for multiple simultaneous keypresses anyway (as they have no diodes), and we 
        // probably don't want to fire off tons of network calls all at once.
        if (didHandleEvent) {
            LOG("Event already handled, ignoring key press for key '");
            LOG(key);
            LOGLN("'");
            continue;
        }

        if(event == KEY_JUST_PRESSED) {
            LOG("'");
            LOG(key);
            LOGLN("' key pressed");

            // First see if there are any toggles assigned to the key.
            didHandleEvent = handleToggles(key);

            // If not, try the macros.
            if (!didHandleEvent) {
                didHandleEvent = handleMacros(key);
            }

            // No actions assigned to this key.
            if (!didHandleEvent) {
                LOG("No assignment for key '");
                LOG(key);
                LOGLN("'");
            }
        } 
    }
}

bool handleToggles(char key) {
    for (ToggleStates::iterator statesIter = toggleStates.begin(); statesIter != toggleStates.end(); ++statesIter) {
        ToggleState& toggleState = *statesIter;
        const ToggleAction& config = toggleState.config;

        if (key == config.getKey()) {
            const bool newToggleState = !toggleState.toggleState;

            activityFlash();
            setToggleState(toggleState, newToggleState);

            LOGFMT("Toggled topic '%s', value: %s\n", config.getToggleTopic().c_str(), newToggleState ? "1" : "0");
            return true;
        }
    }    

    return false;
}

bool handleMacros(char key) {
    for (int i = 0; i < macroConfigCount; i++) {
        const MacroAction& config = macroConfigs[i];

        if (key == config.getKey()) {
            const MacroAction::Commands& commands = config.getCommands();

            for (MacroAction::Commands::const_iterator commandsIter = commands.begin(); commandsIter != commands.end(); ++commandsIter) {
                const MacroAction::Command& command = *commandsIter;
                mqttClient.publish(command.topic, command.message);

                LOGFMT("Executed macro for topic '%s', message: %s\n", command.topic, command.message);
            }

            return true;
        }
    }

    return false;
}

ToggleState* getToggleStateByTopic(const String& topic) {
    for (ToggleStates::iterator statesIter = toggleStates.begin(); statesIter != toggleStates.end(); ++statesIter) {
        ToggleState& toggleState = *statesIter;
        const ToggleAction& config = toggleState.config;

        if (config.getToggleTopic() == topic) {
            return &toggleState;
        }
    }    

    return nullptr;
}

void setToggleState(ToggleState& toggleState, bool toggled) {
    const char* payload = toggled ? "1" : "0";

    toggleState.isBusy = true;
    toggleState.toggleState = toggled;
    mqttClient.publish(toggleState.config.getToggleTopic(), payload, true);
}

// ------------------------------
// Status LED
// ------------------------------
void initStatusLED() {
    pinMode(PIN_STATUS_LED_POWER, OUTPUT);
    digitalWrite(PIN_STATUS_LED_POWER, HIGH);

    statusLED.begin();
    statusLED.show();
}

void updateStatusLED() {
    switch (connectionState) {
        case ConnectionState::connecting:
            setStatusLEDColor(STATUS_COLOR_BOOT, STATUS_LED_NORMAL_BRIGHTNESS);
            break;

        case ConnectionState::connected:
            if (isAnyToggleBusy() || activityFlashTimer > 0) {
                setStatusLEDColor(STATUS_COLOR_BUSY, STATUS_LED_NORMAL_BRIGHTNESS);
            } else {
                uint8_t brightness = STATUS_LED_NORMAL_BRIGHTNESS;

                // Dim if it's almost time to go to sleep.
                if (timeSinceUserActivity >= USER_INACTIVITY_TIMEOUT - SLEEP_WARN_INTERVAL) {
                    brightness = STATUS_LED_DIM_BRIGHTNESS;
                } 

                // Pulse the LED to indicate the remote is plugged in to power.
                 if (isConnectedToUSB) {
                    uint8_t pulse = Color::sine8(millis() / 16);
                    brightness = Color::scale8(pulse, brightness);

                    // Don't let the brightness get all the way to 0.
                    const uint8_t minBrightness = 4;
                    brightness = max(brightness, minBrightness);
                } 

                setStatusLEDColor(STATUS_COLOR_CONNECTED, brightness);
            }
            break;

        case ConnectionState::disconnected:
            setStatusLEDColor(STATUS_COLOR_DISCONNECTED, STATUS_LED_NORMAL_BRIGHTNESS);
            break;

        default:
            break;
    }
}

void setStatusLEDColor(uint32_t c, uint8_t brightness) {
    c = Color::gamma32(c);
    c = Color::scale(c, brightness);

    statusLED.fill(c);
    statusLED.show();
}

// ------------------------------
// Activity tracking/deep sleep
// ------------------------------
void activityFlash() {
    activityFlashTimer = activityFlashDuration;
}

void userDidInteract() {
    timeSinceUserActivity = 0;
    LOGLN("Activity detected, resetting timer");
}

void deepSleep() {
    // Remember the current state of the tilt sensor.
    // We want to wake when the state changes.
    uint8_t wakeState = (tiltState) ? 0 : 1;
    LOGFMT("Going to sleep. Current tilt switch state is %d\n", tiltState);
    LOGFMT("Will wake on tilt switch state: %d\n", wakeState);
    
    // Cut power to the status LED
    pinMode(PIN_STATUS_LED_POWER, OUTPUT);
    digitalWrite(PIN_STATUS_LED_POWER, LOW);

    // Goodnight!
    esp_sleep_enable_ext0_wakeup(GPIO_WAKE, wakeState);
    rtc_gpio_hold_en(GPIO_WAKE);
    esp_deep_sleep_start();
}

void logWakeReason() {
    esp_sleep_wakeup_cause_t wakeReason = esp_sleep_get_wakeup_cause();

    switch(wakeReason) {
        case ESP_SLEEP_WAKEUP_EXT0: 
            LOGLN("Wakeup caused by external signal using RTC_IO"); 
            break;

        case ESP_SLEEP_WAKEUP_EXT1:
            LOGLN("Wakeup caused by external signal using RTC_CNTL"); 
            break;

        case ESP_SLEEP_WAKEUP_TIMER: 
            LOGLN("Wakeup caused by timer"); 
            break;

        case ESP_SLEEP_WAKEUP_TOUCHPAD: 
            LOGLN("Wakeup caused by touchpad"); 
            break;

        case ESP_SLEEP_WAKEUP_ULP: 
            LOGLN("Wakeup caused by ULP program"); 
            break;

        default: 
            LOGFMT("Wakeup was not caused by deep sleep: %d\n", wakeReason); 
            break;
    }
}

// ------------------------------
// Voltage monitor
// ------------------------------
void initVoltageMonitor() {
    pinMode(PIN_VOLTAGE, INPUT);
    readVoltage();
}

void readVoltage() {
    rawVoltage = analogRead(PIN_VOLTAGE);

    // float voltage = rawVoltage * 5.0 / (1<<13);
    // With a 13-bit ADC, and a max voltage of 5.0, 6554 is about 4.0 volts.
    // If the voltage is above 4 volts we can safely assume we're plugged in.
    isConnectedToUSB = (rawVoltage >= 6554);
}
