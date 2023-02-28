
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


/*******************************************************************************
 * Function Name: handle_error
 ********************************************************************************
 * Summary:
 * User defined error handling function.
 *
 *******************************************************************************/
void handle_error(void) {
	/* Disable all interrupts */
	__disable_irq();

	Cy_GPIO_Write(CYBSP_LED_RGB_RED_PORT, CYBSP_LED_RGB_RED_NUM,
	CYBSP_LED_STATE_ON);
	/* Switch On error LED */

	while (1) {
	}
}
