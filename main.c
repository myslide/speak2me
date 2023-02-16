/******************************************************************************
 * File Name: main.c
 * author:mysli
 * version 20230215
 *
 * Description: This is the source code for the PSoC 4 CapSense CSD Touchpad
 * controlled MP3 player for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"
#include "DFPlayer.h"
#include <stdbool.h>

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define CAPSENSE_INTR_PRIORITY    (3u)
#define CY_ASSERT_FAILED          (0u)
#define GENERALHELPNUMBER       (1u)
/*******************************************************************************
 * Global Definitions
 *******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;

#if CY_CAPSENSE_BIST_EN
/* Variables to hold sensor parasitic capacitances */
uint32_t sensor_cp = 0;
cy_en_capsense_bist_status_t status;
#endif /* CY_CAPSENSE_BIST_EN */

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static void initialize_capsense(void);
static void capsense_isr(void);

static volatile bool isPlayGeneralHelp = false;
void sayGlobalDeviceHelp() {
	isPlayGeneralHelp = true;
	if (Cy_GPIO_Read(CYBSP_MP3BUSY_PORT, CYBSP_MP3BUSY_PIN) != 0) {
		dfp_play(GENERALHELPNUMBER);
		delay(100);
	}
//	while (Cy_GPIO_Read(CYBSP_MP3BUSY_PORT, CYBSP_MP3BUSY_PIN) == 0) {
//		//delay(300);
//	}

}
static void sayMemoryHelp(int helpnumber) {

	delay(500);//suppress double calls from a interrupts
	if (Cy_GPIO_Read(CYBSP_MP3BUSY_PORT, CYBSP_MP3BUSY_PIN) != 0) {
		dfp_play(helpnumber);
		delay(100);
	}
	while (Cy_GPIO_Read(CYBSP_MP3BUSY_PORT, CYBSP_MP3BUSY_PIN) == 0) {
		//delay(300);
	}

}

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CapSense
 *  - initialize tuner communication
 *  - perform Cp measurement if Built-in Self test (BIST) is enabled
 *  - scan touch input continuously
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void) {
	cy_rslt_t result = CY_RSLT_SUCCESS;
	cy_en_scb_uart_status_t initstatus;

	/* Initialize the device and board peripherals */
	result = cybsp_init();

	/* Board init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS) {
		CY_ASSERT(CY_ASSERT_FAILED);
	}
	/* Enable global interrupts */
	__enable_irq();
	/* Initialize the UART */
	initstatus = dfp_init(SCB1, &CYBSP_UART1_config);
	/* Initialize CapSense */
	initialize_capsense();

	cy_stc_capsense_touch_t *touchinfo;
	/* Initialization failed. Handle error */
	if (initstatus != CY_SCB_UART_SUCCESS) {
		handle_error();
	}

	Cy_GPIO_Write(CYBSP_LED_RGB_RED_PORT, CYBSP_LED_RGB_RED_NUM,
	CYBSP_LED_STATE_ON);
	delay(1000);
	Cy_GPIO_Write(CYBSP_LED_RGB_RED_PORT, CYBSP_LED_RGB_RED_NUM,
	CYBSP_LED_STATE_OFF);
	/* Start the first scan */
	Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
	dfp_volume(25);

	cy_capsense_status_t status;
	for (;;) {
		if (CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context)) {
			/* Process all widgets */
			Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

			if (0 != Cy_CapSense_IsWidgetActive(
			CY_CAPSENSE_TOUCHPAD0_WDGT_ID, &cy_capsense_context)) {
				if(isPlayGeneralHelp){
					dfp_stop();
					isPlayGeneralHelp = false;
				}
				if (0 != Cy_CapSense_IsSensorActive(
				CY_CAPSENSE_TOUCHPAD0_WDGT_ID,
				CY_CAPSENSE_TOUCHPAD0_COL0_ID, &cy_capsense_context)) {
					Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM,
					CYBSP_LED_STATE_ON);
					sayMemoryHelp(2);
					Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM,
					CYBSP_LED_STATE_OFF);
				}
				if (0 != Cy_CapSense_IsSensorActive(
				CY_CAPSENSE_TOUCHPAD0_WDGT_ID,
				CY_CAPSENSE_TOUCHPAD0_COL3_ID, &cy_capsense_context)) {
					Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM,
					CYBSP_LED_STATE_ON);
					sayMemoryHelp(3);
					Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM,
					CYBSP_LED_STATE_OFF);
				}

				if (0 != Cy_CapSense_IsSensorActive(
				CY_CAPSENSE_TOUCHPAD0_WDGT_ID,
				CY_CAPSENSE_TOUCHPAD0_COL6_ID, &cy_capsense_context)) {
					Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM,
					CYBSP_LED_STATE_ON);
					sayMemoryHelp(4);
					Cy_GPIO_Write(CYBSP_LED_BTN1_PORT, CYBSP_LED_BTN1_NUM,
					CYBSP_LED_STATE_OFF);
				}
			} else {
				if (0 != Cy_CapSense_IsProximitySensorActive(
				CY_CAPSENSE_PROXIMITY0_WDGT_ID,
				CY_CAPSENSE_PROXIMITY0_SNS0_ID, &cy_capsense_context)) {
					//delay(500);
//				Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
//				if (CY_CAPSENSE_NOT_BUSY
//						== Cy_CapSense_IsBusy(&cy_capsense_context)) {
//					Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);
//					if (0 == Cy_CapSense_IsWidgetActive(
//					CY_CAPSENSE_TOUCHPAD0_WDGT_ID, &cy_capsense_context)) {

					Cy_GPIO_Write(CYBSP_LED_RGB_RED_PORT,
					CYBSP_LED_RGB_RED_NUM,
					CYBSP_LED_STATE_ON);
					sayGlobalDeviceHelp();
					Cy_GPIO_Write(CYBSP_LED_RGB_RED_PORT,
					CYBSP_LED_RGB_RED_NUM,
					CYBSP_LED_STATE_OFF);
				}

			}

			/* Start the next scan */
			Cy_CapSense_ScanAllWidgets(&cy_capsense_context);
		}
	}

}

/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CapSense and configures the CapSense
 *  interrupt.
 *
 *******************************************************************************/
static void initialize_capsense(void) {
	cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

	/* CapSense interrupt configuration */
	const cy_stc_sysint_t capsense_interrupt_config = { .intrSrc =
	CYBSP_CSD_IRQ, .intrPriority = CAPSENSE_INTR_PRIORITY, };

	/* Capture the CSD HW block and initialize it to the default state. */
	status = Cy_CapSense_Init(&cy_capsense_context);

	if (CY_CAPSENSE_STATUS_SUCCESS == status) {
		/* Initialize CapSense interrupt */
		Cy_SysInt_Init(&capsense_interrupt_config, capsense_isr);
		NVIC_ClearPendingIRQ(capsense_interrupt_config.intrSrc);
		NVIC_EnableIRQ(capsense_interrupt_config.intrSrc);

		/* Initialize the CapSense firmware modules. */
		status = Cy_CapSense_Enable(&cy_capsense_context);
	}

	if (status != CY_CAPSENSE_STATUS_SUCCESS) {
		/* This status could fail before tuning the sensors correctly.
		 * Ensure that this function passes after the CapSense sensors are tuned
		 * as per procedure give in the Readme.md file */
	}
}

/*******************************************************************************
 * Function Name: capsense_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense block.
 *
 *******************************************************************************/
static void capsense_isr(void) {
	Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}

#if CY_CAPSENSE_BIST_EN
	/*******************************************************************************
	 * Function Name: measure_sensor_cp
	 ********************************************************************************
	 * Summary:
	 *  Measures the self capacitance of the sensor electrode (Cp) in Femto Farad and
	 *  stores its value in the variable sensor_cp.
	 *
	 *******************************************************************************/
	static void measure_sensor_cp(void) {
		/* Measure the self capacitance of sensor electrode */
		status = Cy_CapSense_MeasureCapacitanceSensor(
				CY_CAPSENSE_BUTTON0_WDGT_ID,
				CY_CAPSENSE_BUTTON0_SNS0_ID, &sensor_cp, &cy_capsense_context);
	}
#endif /* CY_CAPSENSE_BIST_EN */
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

/* [] END OF FILE */
