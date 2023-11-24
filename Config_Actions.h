// Only include in main file

#include "ActionTypes.h"
#include "AdafruitIO_Feed_Topic.h"

// ------------------------------
// Toggles
// ------------------------------
const ToggleAction toggleConfigs[] = {
    ToggleAction('1', AIO_FEED_TOPIC(test-led)),
};

// ------------------------------
// Macros
// ------------------------------
const MacroAction macroConfigs[] = {
    MacroAction('2', {
        {AIO_FEED_TOPIC(test-stream-1), "2 key pressed"}, 
    }),

    MacroAction('3', {
        {AIO_FEED_TOPIC(test-stream-1), "3 key pressed"}, 
        {AIO_FEED_TOPIC(test-stream-2), "3 key pressed"}, 
    }),
};

// ------------------------------
// Auto-calculate counts based on numer of entries in each array.
// ------------------------------
const int toggleConfigCount = sizeof(toggleConfigs) / sizeof(toggleConfigs[0]);
const int macroConfigCount = sizeof(macroConfigs) / sizeof(macroConfigs[0]);
