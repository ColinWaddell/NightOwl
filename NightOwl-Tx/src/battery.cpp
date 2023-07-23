#include <Arduino.h>

#include "config.h"

float battery_voltage() {
    float bat_v;

    bat_v = analogRead(VBATPIN);
    bat_v *= 2;    /* v-divider halfs voltage */
    bat_v *= 3.3;  /* Multiply by 3.3V, our reference voltage */
    bat_v /= 1024; /* convert to voltage */

    return bat_v;
}