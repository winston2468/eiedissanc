/*
 * GPIOInterface.c
 *
 *  Created on: 13 Oct 2021
 *      Author: Winston
 */
#include <services/gpio/adi_gpio.h>
#include <stdio.h>
#include "anc_test1.h"
#include <services/int/adi_int.h>
#define GPIO_MEMORY_SIZE (ADI_GPIO_CALLBACK_MEM_SIZE*2)

static uint32_t gpioMemory[GPIO_MEMORY_SIZE];
extern void pinIntCallback(ADI_GPIO_PIN_INTERRUPT ePinInt, uint32_t PinIntData,
		void *pCBParam);

static void CheckGpioResult(ADI_GPIO_RESULT result) {
	if (result != ADI_GPIO_SUCCESS) {
		printf("GPIO failure\n");
	}
}

void configGpio(void) {
	int i;
	uint32_t gpioMaxCallbacks;
	ADI_INT_STATUS eIntResult;

	// init the GPIO service
	ADI_GPIO_RESULT result = adi_gpio_Init((void*) gpioMemory,
	GPIO_MEMORY_SIZE, &gpioMaxCallbacks);
	CheckGpioResult(result);

	/*
	 * Setup Push Button 1
	 */

	// set GPIO input
	result = adi_gpio_SetDirection(
	PUSH_BUTTON1_PORT,
	PUSH_BUTTON1_PIN, ADI_GPIO_DIRECTION_INPUT);
	CheckGpioResult(result);

	// set edge sense mode (PORT B is connected to Pin Interrupt 0)
	result = adi_gpio_SetPinIntEdgeSense(
	PUSH_BUTTON1_PINT,
	PUSH_BUTTON1_PINT_PIN, ADI_GPIO_SENSE_RISING_EDGE);
	CheckGpioResult(result);

	// register pinint callback
	result = adi_gpio_RegisterCallback(
	PUSH_BUTTON1_PINT,
	PUSH_BUTTON1_PINT_PIN, pinIntCallback, (void*) 0);
	CheckGpioResult(result);

	// set pin interrupt mask
	result = adi_gpio_EnablePinInterruptMask(
	PUSH_BUTTON1_PINT,
	PUSH_BUTTON1_PINT_PIN, true);
	CheckGpioResult(result);

	/*
	 * Setup Push Button 2
	 */

	// set GPIO input
	result = adi_gpio_SetDirection(
	PUSH_BUTTON2_PORT,
	PUSH_BUTTON2_PIN, ADI_GPIO_DIRECTION_INPUT);
	CheckGpioResult(result);

	// set edge sense mode (PORT E is connected to Pin Interrupt 3)
	result = adi_gpio_SetPinIntEdgeSense(
	PUSH_BUTTON2_PINT,
	PUSH_BUTTON2_PINT_PIN, ADI_GPIO_SENSE_RISING_EDGE);
	CheckGpioResult(result);

	// register pinint 3 callback
	result = adi_gpio_RegisterCallback(
	PUSH_BUTTON2_PINT,
	PUSH_BUTTON2_PINT_PIN, pinIntCallback, (void*) 0);
	CheckGpioResult(result);

	// set pin interrupt mask
	result = adi_gpio_EnablePinInterruptMask(
	PUSH_BUTTON2_PINT,
	PUSH_BUTTON2_PINT_PIN, true);
	CheckGpioResult(result);

	/*
	 * Setup LED's
	 */

	// set GPIO output LED
	result = adi_gpio_SetDirection(
	LED1_PORT,
	LED1_PIN, ADI_GPIO_DIRECTION_OUTPUT);
	CheckGpioResult(result);

	// set GPIO output LED
	result = adi_gpio_SetDirection(
	LED2_PORT,
	LED2_PIN, ADI_GPIO_DIRECTION_OUTPUT);
	CheckGpioResult(result);

	//ADAU1979 & ADAU1962a GPIO setup

	result = adi_gpio_SetDirection(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14,
			ADI_GPIO_DIRECTION_OUTPUT);
	CheckGpioResult(result);

	/* bring reset low */
	result = adi_gpio_Clear(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14);
	CheckGpioResult(result);

	/* delay */
	for (i = DELAY_COUNT; i; i--)
		;

	/* bring reset high */
	result = adi_gpio_Set(ADI_GPIO_PORT_A, ADI_GPIO_PIN_14);
	CheckGpioResult(result);
	/* delay */
	for (i = DELAY_COUNT; i; i--)
		;
}
