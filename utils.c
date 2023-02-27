
#include "utils.h"
/**
 * counts x Green RGB LED blink
 */
void blinkGreen(uint16_t counts) {
	int i = 0;
	while (i++ < counts) {
		Cy_GPIO_Write(CYBSP_LED_RGB_GREEN_PORT, CYBSP_LED_RGB_GREEN_NUM,
		CYBSP_LED_STATE_ON);
		delay(500);
		Cy_GPIO_Write(CYBSP_LED_RGB_GREEN_PORT, CYBSP_LED_RGB_GREEN_NUM,
		CYBSP_LED_STATE_OFF);
		delay(500);
	}
}

void greenLEDOn(){
	Cy_GPIO_Write(CYBSP_LED_RGB_GREEN_PORT,
						CYBSP_LED_RGB_GREEN_NUM,
						CYBSP_LED_STATE_ON);
}

void greenLEDOff(){
	Cy_GPIO_Write(CYBSP_LED_RGB_GREEN_PORT,
						CYBSP_LED_RGB_GREEN_NUM,
						CYBSP_LED_STATE_OFF);
}