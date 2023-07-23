#include <Adafruit_SleepyDog.h>
#include <Arduino.h>

#include "battery.h"
#include "blinks.h"
#include "config.h"
#include "door.h"
#include "luminance.h"
#include "radio.h"

/**********************************************************
 * Board Setup
 *********************************************************/
void setup() {
    /* Enable Watchdog */
    (void)Watchdog.enable(WDT_MS);

    /* Debug output */
    Serial.begin(115200);

    /* Initialise hardware */
    blink_init();
    radio_init();
    door_init();

    /* Power management */
#if LOW_POWER_SLEEP
    Serial.end();
    USBDevice.detach();
#endif
}

/**********************************************************
 * Super-loop
 *********************************************************/
void loop() {
    static bool first_loop = true;
    static bool door_open_previously = false;
    bool perform_tx = false;
    bool door_open = false;
    blink_code status = BLINK_NOTHING_HAPPENED;

    door_open = door_is_open();

    if (!first_loop) {
        /* Tx a packet if the door has changed state and
         * there is enough light
        */
        if (door_open_previously != door_open) {
            status = BLINK_DOOR_CHANGED;
            if (luminance_read() < LUMINANCE_THRESHOLD) {
                perform_tx = true;
            }
        }
    }
    else {
        /* Always Tx on boot */
        perform_tx = true;
        status = BLINK_FIRST_LOOP;
    }

    /* Tx a packet if it's required */
    if (perform_tx) {
        /* Build packet */
        rf_packet packet = {
            .door_open = door_open
        };

        /* Transmit */
        if (radio_tx(&packet)) {
            status = BLINK_TX_SUCCESS;
        }
        else {
            status = BLINK_TX_FAILURE;
#if VERBOSE_DEBUG
            Serial.println("Broadcast failed");
#endif
        }
    }

#if STATUS_BLINKS
    blink_led(status);
#else
    blink_led(BLINK_NOTHING_HAPPENED);
#endif

#if VERBOSE_DEBUG
    Serial.printf("Door: %d, Luminance: %ld, Battery: ", door_open, luminance_read());
    Serial.println(battery_voltage());
#endif

    /* Tidy up operations */
    radio_sleep();
    first_loop = false;
    door_open_previously = door_open;

    /* Sleep till next loop */
#if LOW_POWER_SLEEP /* Put device to sleep */
    Watchdog.sleep(SLEEP_MS);
#else  /* Regular delay */
    delay(SLEEP_MS);
#endif /* LOW_POWER_SLEEP */

    /* WDT Kick */
    Watchdog.reset();
    Watchdog.enable(WDT_MS); /* reenable required after sleep */
}
