#ifndef __CONFIG_PREFERENCES_H__
#define __CONFIG_PREFERENCES_H__

// Throttle the loop update a bit, we don't need to update so often.
// This also provides de-facto debouncing for the tilt switch and keypad.
#define LOOP_DELAY 10

// WiFi connection attempt will be made for CONNECT_TIMEOUT milliseconds beforing timing out.
#define CONNECT_TIMEOUT             (20 * 1000u)

// When disconnected, if connection is not re-established after this time, the remote will sleep.
#define RECONNECT_TIMEOUT           (10 * 1000u)

// Remote will enter deep sleep afer USER_INACTIVITY_TIMEOUT milliseconds of user inactivity.
#define USER_INACTIVITY_TIMEOUT     (45 * 1000u)

// Status LED will dim SLEEP_WARN_INTERVAL milliseconds before putting the remote to sleep.
#define SLEEP_WARN_INTERVAL          (10 * 1000u)

// Status LED brightness (0-255)
#define STATUS_LED_NORMAL_BRIGHTNESS    20
#define STATUS_LED_DIM_BRIGHTNESS       10

// Status colors
#define STATUS_COLOR_BOOT          0x0000FF
#define STATUS_COLOR_BUSY          0xFFFFFF
#define STATUS_COLOR_CONNECTED     0x00FF00
#define STATUS_COLOR_DISCONNECTED  0xFF0000

#endif