/*********************************************************************************

Copyright(c) 2015 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/
/*
* Use the GPIO service to Toggle LEDs when the push buttons are pressed on the
* EZ-Kit (callback method).
*/

#include <services/gpio/adi_gpio.h>
#include <services/int/adi_int.h>
#include <stdio.h>
#include "Button_LED_GPIO.h"

/* Managed drivers and/or services include */
#include "adi_initialize.h"

#define GPIO_MEMORY_SIZE (ADI_GPIO_CALLBACK_MEM_SIZE*2)

/* Macro for reporting errors */
#define REPORT_ERROR printf

/* used for exit timeout */
//#define MAXCOUNT (0x7FFFFFFFu)

static uint8_t gpioMemory[GPIO_MEMORY_SIZE];

static bool bError;
//static volatile uint64_t count;

extern void ConfigSoftSwitches(void);

static void CheckResult(ADI_GPIO_RESULT result)
{
    if (result != ADI_GPIO_SUCCESS) {
        REPORT_ERROR("GPIO failure\n");
        bError = true;
    }
}

extern void gpioCallback(ADI_GPIO_PIN_INTERRUPT ePinInt, uint32_t Data, void *pCBParam);


int button_led_gpio_init(void)
{
    uint32_t gpioMaxCallbacks;
    ADI_GPIO_RESULT result;

    /* initialize the GPIO service */
    result = adi_gpio_Init(
            (void*)gpioMemory,
            GPIO_MEMORY_SIZE,
            &gpioMaxCallbacks);
    CheckResult(result);
//
//    /* Initialize managed drivers and/or services */
//    adi_initComponents();
//
//	/* Software Switch Configuration */
//	ConfigSoftSwitches();

    /*
     * Setup Push Button 1
     */

    /* set GPIO input */
    result = adi_gpio_SetDirection(
        PUSH_BUTTON1_PORT,
        PUSH_BUTTON1_PIN,
        ADI_GPIO_DIRECTION_INPUT);
    CheckResult(result);

    /* set input edge sense */
	result = adi_gpio_SetPinIntEdgeSense(
		PUSH_BUTTON1_PINT,
		PUSH_BUTTON1_PINT_PIN,
	    ADI_GPIO_SENSE_RISING_EDGE);
    CheckResult(result);

    /* register gpio callback */
    result = adi_gpio_RegisterCallback(
    	PUSH_BUTTON1_PINT,
    	PUSH_BUTTON1_PINT_PIN,
    	gpioCallback,
   	    (void*)0);
    CheckResult(result);

	/* enable interrupt mask */
	result = adi_gpio_EnablePinInterruptMask(
		PUSH_BUTTON1_PINT,
		PUSH_BUTTON1_PINT_PIN,
	    true);
	CheckResult(result);

    /*
     * Setup Push Button 2
     */

    /* set GPIO input */
    result = adi_gpio_SetDirection(
        PUSH_BUTTON2_PORT,
        PUSH_BUTTON2_PIN,
        ADI_GPIO_DIRECTION_INPUT);
    CheckResult(result);

    /* set input edge sense */
	result = adi_gpio_SetPinIntEdgeSense(
		PUSH_BUTTON2_PINT,
		PUSH_BUTTON2_PINT_PIN,
	    ADI_GPIO_SENSE_RISING_EDGE);
    CheckResult(result);

    /* register gpio callback */
    result = adi_gpio_RegisterCallback(
    	PUSH_BUTTON2_PINT,
    	PUSH_BUTTON2_PINT_PIN,
    	gpioCallback,
   	    (void*)0);
    CheckResult(result);

	/* enable interrupt mask */
	result = adi_gpio_EnablePinInterruptMask(
		PUSH_BUTTON2_PINT,
		PUSH_BUTTON2_PINT_PIN,
	    true);
	CheckResult(result);

    /*
     * Setup LEDs
     */

    /* set GPIO output LED 0 */
    result = adi_gpio_SetDirection(
        LED1_PORT,
        LED1_PIN,
        ADI_GPIO_DIRECTION_OUTPUT);
    CheckResult(result);

    /* set GPIO output LED 1 */
    result = adi_gpio_SetDirection(
        LED2_PORT,
        LED2_PIN,
        ADI_GPIO_DIRECTION_OUTPUT);
    CheckResult(result);

//    count = 0u;
//    printf("\nPress %s or %s to toggle LEDs on the EZ-Kit \n\n", PUSH_BUTTON1_LABEL, PUSH_BUTTON2_LABEL);
//
//    /* wait for push button interrupts - exit the loop after a while */
//    while(count < MAXCOUNT)
//    {
//        count++;
//    }
//
//    if (bError) {
//        printf("Error configuring the push buttons or LEDs\n");
//    } else {
//        printf("All done\n");
//    }

    return 0;
}


