#include "samd51.h"  // if not already included

void delay(uint32_t cycles) {
    uint32_t num = cycles*10;
    while (num--) {
        __NOP();
    }
}

