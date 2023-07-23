#ifndef BLINKS_H
#define BLINKS_H

typedef enum {
    BLINK_FIRST_LOOP,
    BLINK_NOTHING_HAPPENED,
    BLINK_DOOR_CHANGED,
    BLINK_TX_SUCCESS,
    BLINK_TX_FAILURE,
    BLINK_CODES_ERROR /* Must be last entry */
} blink_code;

void blink_init();
void blink_led(blink_code status);

#endif