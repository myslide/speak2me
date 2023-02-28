/******************************************************************************
 * File Name: main.c
 * author:mysli
 * version 20230227
 *
 *Description: This is the source code for the PSoC 4 CapSense CSD Touchpad
 * controlled MP3 player for ModusToolbox.
 *
 * Related Document: See https://www.hackster.io/myslide/speak2me-bd993e
 *
 * Used PINS of CY8CKIT-041-41XX:
 *
 * RINGDETECTOR - P0_2, D7@J4, DigitalIn
 * SWITCHHOOK   - P0_7, 12@J2, DigitalIn
 * RELAY - P2_7,D9@J4 ,DigitalOut
 * MP3BUSY - P2_5,A5@J2 ,DigitalIn
 * CYBSP_DEBUG_UART_TX - P0_5,D1@J4 ,DigitalOut,UART
 *
 * CYBSP_CSD_COLx
 *
 *
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
#include "utils.h"
#include <stdbool.h>

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define CAPSENSE_INTR_PRIORITY    (3u)
#define CY_ASSERT_FAILED          (0u)
#define GENERALHELPNUMBER       (1u)

/*****************************************************************************
 * Macros 4 WDT
 *****************************************************************************/
/* WDC Interrupt flag states */
#define WDC_INTERRUPT_FLAG_CLEAR    0
#define WDC_INTERRUPT_FLAG_SET      1

/* ILO Compensation flag states */
#define ILO_COMP_FLAG_CLEAR         0
#define ILO_COMP_FLAG_SET           1

/* Define COUNTER0 delay in microseconds */
#define COUNTER0_DELAY_US           (250 * 1000)

/*****************************************************************************
 * Macros 4 Handset switch
 *****************************************************************************/

#define DELAY_SHORT             (250)   /* milliseconds */
#define DELAY_LONG              (500)   /* milliseconds */

#define LED_BLINK_COUNT         (4)

#define SWITCH_INTR_PRIORITY    (3u)

/*******************************************************************************
 * Global Definitions
 *******************************************************************************/

#if CY_CAPSENSE_BIST_EN
/* Variables to hold sensor parasitic capacitances */
uint32_t sensor_cp = 0;
cy_en_capsense_bist_status_t status;
#endif /* CY_CAPSENSE_BIST_EN */

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* WDC Interrupt flag */
volatile uint32_t interrupt_flag = 0;
/*The handset switch interrupt flag*/
volatile uint32_t switch_interrupt_flag = false;
volatile uint32_t isPhoneOn = false;
//to avoid endless loops, CAPSENSE can be blocked for a certain time
volatile uint32_t isCSDreleased = true;
volatile uint32_t isPickedUp = false;
volatile uint32_t isIncoming = false;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static void initialize_capsense(void);
static void capsense_isr(void);
void switch_ISR();
void initPhone();

/* WDT initialization function */
void wdt_init(void);
/* WDT interrupt service routine */
void wdt_isr(void);
void wdc_interrupt_handler(void);

/*****************************************************************************
 * Global Variables
 *****************************************************************************/

static volatile bool isPlayGeneralHelp = false;
void sayGlobalDeviceHelp() {
	isPlayGeneralHelp = true;
	if (Cy_GPIO_Read(MP3BUSY_PORT, MP3BUSY_PIN) != 0) {
		dfp_play(GENERALHELPNUMBER);
		delay(100);
	}
//	while (Cy_GPIO_Read(CYBSP_MP3BUSY_PORT, CYBSP_MP3BUSY_PIN) == 0) {
//		//delay(300);
//	}

}
static void sayMemoryHelp(int helpnumber) {

	delay(500); //suppress double calls from a interrupts
	if (Cy_GPIO_Read(MP3BUSY_PORT, MP3BUSY_PIN) != 0) {
		dfp_play(helpnumber);
		delay(100);
	}
	while (Cy_GPIO_Read(MP3BUSY_PORT, MP3BUSY_PIN) == 0) {
		//delay(300);
	}

}

/******************************************************************************
 * Switch interrupt configuration structure
 *****************************************************************************/
const cy_stc_sysint_t switch_intr_config = { .intrSrc = SWITCHHOOK_IRQ, /* Source of interrupt signal */
.intrPriority = SWITCH_INTR_PRIORITY /* Interrupt priority */
};

/*******************************************************************************
 * Function Name: Switch_ISR
 ********************************************************************************
 *
 * Summary:
 *  This function is executed when GPIO interrupt is triggered.
 *
 *******************************************************************************/
void switch_ISR() {
	/* Clears the triggered pin interrupt */

	isIncoming = Cy_GPIO_Read(RINGDETECTOR_PORT, RINGDETECTOR_NUM);
	switch_interrupt_flag = false;
	if (isIncoming != false) {
		initPhone();
		blinkGreen(6);
		switch_interrupt_flag = false;
	} else {
		/* Set interrupt flag */
		delay(20);
		switch_interrupt_flag = true;
	}
	//clear IRT it after processing for debounce
	Cy_GPIO_ClearInterrupt(SWITCHHOOK_PORT, SWITCHHOOK_NUM);
	Cy_GPIO_ClearInterrupt(RINGDETECTOR_PORT, RINGDETECTOR_NUM);
	NVIC_ClearPendingIRQ(switch_intr_config.intrSrc);
}

/**
 * After the PickUp interrupt, switch to listen for the hang up interrupt.
 * switch_interrupt_flag will be set to false
 */
void toggleSwitchHookISRIndicator() {
	if (CY_GPIO_INTR_RISING == Cy_GPIO_GetInterruptEdge(SWITCHHOOK_PORT,
	SWITCHHOOK_NUM)) {
		Cy_GPIO_SetInterruptEdge(SWITCHHOOK_PORT, SWITCHHOOK_NUM,
		CY_GPIO_INTR_FALLING);
	} else {
		Cy_GPIO_SetInterruptEdge(SWITCHHOOK_PORT, SWITCHHOOK_NUM,
		CY_GPIO_INTR_RISING);
	}
}

/*******************************************************************************
 * Function Name: wdc_interrupt_handler
 ********************************************************************************
 * Summary:
 * The wdc interrupt handler function handles the WDC interrupt. The interrupt
 * flag is set depending on the interrupt status and counter mask. The flag is
 * set to perform ILO compensation in case of counter 0.
 *
 *******************************************************************************/
void wdc_interrupt_handler(void) {
	uint32_t status = Cy_WDC_GetInterruptStatus(WCO);

	if ((status & CY_WDC_COUNTER1_Msk) != 0) {
		//disable next line if Counter3 is enabled above
		interrupt_flag = WDC_INTERRUPT_FLAG_SET;
	}
	//enable it for adding Counter 3 are cascaded in the setup
//	if ((status & CY_WDC_COUNTER2_Msk) != 0) {
//		/* Set the WDC interrupt flag */
//		interrupt_flag = WDC_INTERRUPT_FLAG_SET;
//	}
	Cy_WDC_ClearInterrupt(WCO,
			CY_WDC_COUNTER0_Msk | CY_WDC_COUNTER1_Msk/*to use 3 Counters: CY_WDC_COUNTERS_Msk*/);
}

void wdc_init_Interrupt() {

	cy_en_wdc_status_t wdc_Status;
	/* Setup the WDC interrupt */
	(void) Cy_SysInt_SetVector(CYBSP_WDC_IRQ, wdc_interrupt_handler);
	NVIC_EnableIRQ(CYBSP_WDC_IRQ);

	/* Initialize WDC */
	wdc_Status = Cy_WDC_Init(CYBSP_WDC_HW, &CYBSP_WDC_config);
	if (wdc_Status != CY_RSLT_SUCCESS) {
		CY_ASSERT(0);
	}

	/* Enable WDC Counters */
	Cy_WDC_Enable(CYBSP_WDC_HW,
			CY_WDC_COUNTER0_Msk | CY_WDC_COUNTER1_Msk/*to use 3 Counters: CY_WDC_COUNTERS_Msk*/,
			CY_WDC_CLK_ILO_3CYCLES_US);
}
/*
 * The "Thread" to keep the memory buttons light on for rd. 30s
 */
void phoneON() {
	Cy_GPIO_Write(RELAY_PORT, RELAY_NUM, RELAY_ON);
	//wdc_init_Interrupt();
	isPhoneOn = true;
}

void initPhone() {
	phoneON();
	if (isIncoming == true) {
		return;
	}
	sayGlobalDeviceHelp();
}

void shutdownPhone() {
	dfp_stop();
	Cy_GPIO_Write(RELAY_PORT, RELAY_NUM, RELAY_OFF);
	isIncoming = false;
	isPickedUp = false;
	isPhoneOn = false;
}

void phoneOFF() {
	if (interrupt_flag == WDC_INTERRUPT_FLAG_SET) {
		Cy_GPIO_Write(RELAY_PORT, RELAY_NUM, RELAY_OFF);
		/* Clear the interrupt flag */
		interrupt_flag = WDC_INTERRUPT_FLAG_CLEAR;
		isPhoneOn = false;
		//No! maybe handset is not inserted->toggle in switch_ISR() only!
//		Cy_GPIO_SetInterruptEdge(SWITCHHOOK_PORT, SWITCHHOOK_NUM,
//				CY_GPIO_INTR_RISING);
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
	/* Initialize the voice player (DFPlayer) */
	initstatus = dfp_init(SCB1, &CYBSP_UART1_config);
	/* Initialize CapSense */
	initialize_capsense();
	/* Initialization failed. Handle error */
	if (initstatus != CY_SCB_UART_SUCCESS) {
		handle_error();
	}

	/* Initialize and enable GPIO interrupt */
	result = Cy_SysInt_Init(&switch_intr_config, switch_ISR);
	if (result != CY_SYSINT_SUCCESS) {
		CY_ASSERT(0);
	}
	/*Ring indicator model L-H trigger*/
	Cy_GPIO_SetInterruptEdge(RINGDETECTOR_PORT, RINGDETECTOR_NUM,
	CY_GPIO_INTR_RISING);
	/* Make sure we catch the pick up state at first */
	Cy_GPIO_SetInterruptEdge(SWITCHHOOK_PORT, SWITCHHOOK_NUM,
	CY_GPIO_INTR_RISING);
	/* Set the port 0 glitch filter source  */
	Cy_GPIO_SetFilter(SWITCHHOOK_PORT, SWITCHHOOK_NUM);
	Cy_GPIO_SetFilter(RINGDETECTOR_PORT, RINGDETECTOR_NUM);
	/* Clearing and enabling the GPIO interrupt in NVIC */
	NVIC_ClearPendingIRQ(switch_intr_config.intrSrc);
	NVIC_EnableIRQ(switch_intr_config.intrSrc);

	Cy_GPIO_Write(CYBSP_LED_RGB_RED_PORT, CYBSP_LED_RGB_RED_NUM,
	CYBSP_LED_STATE_ON);
	delay(1000);
	Cy_GPIO_Write(CYBSP_LED_RGB_RED_PORT, CYBSP_LED_RGB_RED_NUM,
	CYBSP_LED_STATE_OFF);
	/* Start the first scan */
	Cy_CapSense_ScanAllWidgets(&cy_capsense_context);

	dfp_volume(25);

	for (;;) {
		if (switch_interrupt_flag == true) {
			//handset picked up only!
			delay(20);	//suppress bouncing
			switch_interrupt_flag = false;
			isPickedUp = Cy_GPIO_Read(SWITCHHOOK_PORT,
			SWITCHHOOK_NUM);
			toggleSwitchHookISRIndicator();	//set isPickedUp before further processing
			//hang off
			if ((isPickedUp == true) & (false == isPhoneOn)) {
				initPhone();
			}
			//hang on
			if ((isPickedUp == false) & (true == isPhoneOn)) {
				shutdownPhone();
			}

		}
		//receive a call
		if (isIncoming != false) {
			//ignore the touch supported assisting
			continue;
		}
//		//What is active?Touchpad Cols OR proximity? Prio the Memory buttons->Cols
		if ((isCSDreleased == true)
				& (CY_CAPSENSE_NOT_BUSY
						== Cy_CapSense_IsBusy(&cy_capsense_context))) {
			/* Process all widgets */
			Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

			if (0 != Cy_CapSense_IsWidgetActive(
			CY_CAPSENSE_TOUCHPAD0_WDGT_ID, &cy_capsense_context)) {
				if (isPlayGeneralHelp) {
					dfp_stop();
					isPlayGeneralHelp = false;
				}
				if (0 != Cy_CapSense_IsSensorActive(
				CY_CAPSENSE_TOUCHPAD0_WDGT_ID,
				CY_CAPSENSE_TOUCHPAD0_COL0_ID, &cy_capsense_context)) {
					greenLEDOn();
					if (isPhoneOn == false) {
						phoneON();
					}
					sayMemoryHelp(2);
					greenLEDOff();
				}
				if (0 != Cy_CapSense_IsSensorActive(
				CY_CAPSENSE_TOUCHPAD0_WDGT_ID,
				CY_CAPSENSE_TOUCHPAD0_COL3_ID, &cy_capsense_context)) {
					greenLEDOn();
					if (isPhoneOn == false) {
						phoneON();
					}
					sayMemoryHelp(3);
					greenLEDOff();
				}

				if (0 != Cy_CapSense_IsSensorActive(
				CY_CAPSENSE_TOUCHPAD0_WDGT_ID,
				CY_CAPSENSE_TOUCHPAD0_COL6_ID, &cy_capsense_context)) {
					greenLEDOn();
					if (isPhoneOn == false) {
						phoneON();
					}
					sayMemoryHelp(4);
					greenLEDOff();
				}
			} else {
				if (0 != Cy_CapSense_IsProximitySensorActive(
				CY_CAPSENSE_PROXIMITY0_WDGT_ID,
				CY_CAPSENSE_PROXIMITY0_SNS0_ID, &cy_capsense_context)) {
					if (isPhoneOn == false) {
						initPhone();
					} else {
						sayGlobalDeviceHelp();
					}
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

