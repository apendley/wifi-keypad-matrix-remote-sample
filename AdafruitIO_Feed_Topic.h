#ifndef __ADAFRUIT_IO_FEED_TOPIC_H__
#define __ADAFRUIT_IO_FEED_TOPIC_H__

#include "Config_Secrets.h"

// Macro to make building the Adafruit IO feed topic name easier.
#define AIO_FEED_TOPIC(topic)   IO_USERNAME "/feeds/" #topic

#endif