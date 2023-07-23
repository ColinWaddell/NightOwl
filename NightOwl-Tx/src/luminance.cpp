#include <stdint.h>
#include <wiring_analog.h>

#include "config.h"

uint16_t luminance_read() {
    return 1023 - analogRead(LUMINANCE_AIN);
}