#include <stdint.h>
#include "cycfg.h"
#include "cybsp.h"

/** Pin state for the Available button light LEDS on/Off. */
#define RELAY_ON          (1U)
#define RELAY_OFF         (0U)

void blinkGreen(uint16_t counts);
void greenLEDOn();
void greenLEDOff();
