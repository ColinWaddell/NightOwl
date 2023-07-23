#ifndef CONFIG_H
#define CONFIG_H

/**********************************************************
 * General Config
 *********************************************************/
#define VERBOSE_DEBUG 0
#define LOW_POWER_SLEEP 1 /* USB and Serial disabled after initialisation when true */
#define STATUS_BLINKS 1   /* Different patterns to show status. \
                           *    0: Small single blink only      \
                           *    1: Status codes blinked on led */

#define VBATPIN A7

/**********************************************************
 * Radio Config
 *********************************************************/

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define RF95_FREQ 915.0
#define RF95_TX_PWR 10 /* 5dBm to 23dBm */
#define RF95_NODE_ADDRESS 0

/**********************************************************
 * App
 *********************************************************/
#define SLEEP_MS 3000
#define LUMINANCE_THRESHOLD 50

/**********************************************************
 * Watchdog
 *********************************************************/
#define WDT_MS 2000

#endif