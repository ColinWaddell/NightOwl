#include <Adafruit_SleepyDog.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <variant.h>
#include <wiring_constants.h>

#include "blinks.h"
#include "config.h"

/* Table of blink status codes versus patterns*/
struct _blink_patterns {
    uint32_t duration;
    uint8_t blinks;
} const BLINKS[] = {
    [BLINK_FIRST_LOOP] = { .duration = 500, .blinks = 1 },
    [BLINK_NOTHING_HAPPENED] = { .duration = 10, .blinks = 1 },
    [BLINK_DOOR_CHANGED] = { .duration = 100, .blinks = 2 },
    [BLINK_TX_SUCCESS] = { .duration = 200, .blinks = 4 },
    [BLINK_TX_FAILURE] = { .duration = 200, .blinks = 8 },
    [BLINK_CODES_ERROR] = { .duration = 800, .blinks = 8 },
};

void blink_led(blink_code status) {
    if (status > BLINK_CODES_ERROR) {
        status = BLINK_CODES_ERROR;
    }

    /* Very easy to accidentally overrun
     * the WDT here so remember to give
     * it a kick. Check the two aren't
     * in conflict before blinking.
     */
    assert(BLINKS[status].duration < WDT_MS);

    for (uint8_t i = 0; i < BLINKS[status].blinks; i++) {
        bool last_loop = (i == (BLINKS[status].blinks - 1));

        digitalWrite(LED_BUILTIN, HIGH);
        delay(BLINKS[status].duration);
        Watchdog.reset();

        digitalWrite(LED_BUILTIN, LOW);
        if (!last_loop) {
            delay(BLINKS[status].duration);
            Watchdog.reset();
        }
    }
}