#ifndef CONFIG_H
#define CONFIG_H

/**********************************************************
 * General Config
 *********************************************************/
#define VERBOSE_DEBUG 0
#define WDT_MS 2000
#define SLEEP_MS 3000

/* USB and Serial disabled after initialisation when true */
#define LOW_POWER_SLEEP 1

/**********************************************************
 * Battery
 *********************************************************/
#define VBATPIN A7

/**********************************************************
 * Status Blinks
 *********************************************************/

/* Different patterns to show status.
 *    0: Small single blink only
 *    1: Status codes blinked on led 
 */
#define STATUS_BLINKS 1

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
 * Door Detection
 *********************************************************/
#define DOOR_PIN 12

/**********************************************************
 * Luminance Config
 *********************************************************/
#define LUMINANCE_AIN 3
#define LUMINANCE_THRESHOLD 50

#endif