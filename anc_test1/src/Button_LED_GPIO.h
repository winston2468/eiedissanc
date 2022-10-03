/*********************************************************************************

Copyright(c) 2014-2015 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/

/*!
* @file     Button_LED_GPIO.h
*
* @brief    Primary header file for GPIO service example.
*
* @details  Primary header file for GPIO which contains the processor specific defines.
*
*/


#ifndef _BUTTON_LED_GPIO_H_
#define _BUTTON_LED_GPIO_H_

int button_led_gpio_init();
#if defined(__ADSPSC589__)
/*
 * Push button 1 GPIO settings
 */

/* GPIO port to which push button 1 is connected to */
#define PUSH_BUTTON1_PORT           (ADI_GPIO_PORT_F)

/* GPIO pint to which push button 1 is connected to */
#define PUSH_BUTTON1_PINT           (ADI_GPIO_PIN_INTERRUPT_4)

/* GPIO pin within the port to which push button 1 is connected to */
#define PUSH_BUTTON1_PIN            (ADI_GPIO_PIN_0)

/* pin within the pint to which push button 1 is connected to */
#define PUSH_BUTTON1_PINT_PIN       (ADI_GPIO_PIN_0)

/* Label printed on the EZ-Kit */
#define PUSH_BUTTON1_LABEL          "PB1"

/*
 * Push button 2 GPIO settings
 */

/* GPIO port to which push button 2 is connected to */
#define PUSH_BUTTON2_PORT           (ADI_GPIO_PORT_F)

/* GPIO port to which push button 1 is connected to */
#define PUSH_BUTTON2_PINT           (ADI_GPIO_PIN_INTERRUPT_4)

/* GPIO pin within the port to which push button 1 is connected to */
#define PUSH_BUTTON2_PIN            (ADI_GPIO_PIN_1)

/* pin within the pint to which push button 1 is connected to */
#define PUSH_BUTTON2_PINT_PIN       (ADI_GPIO_PIN_1)

/* Label printed on the EZ-Kit */
#define PUSH_BUTTON2_LABEL          "PB2"


/*
 * LED 1 GPIO settings
 */

/* GPIO port to which LED 1 is connected to */
#define LED1_PORT                   (ADI_GPIO_PORT_E)

/* GPIO pin within the port to which LED 1 is connected to */
#define LED1_PIN                    (ADI_GPIO_PIN_13)


/*
 * LED 2 GPIO settings
 */

/* GPIO port to which LED 2 is connected to */
#define LED2_PORT                   (ADI_GPIO_PORT_E)

/* GPIO pin within the port to which LED 2 is connected to */
#define LED2_PIN                    (ADI_GPIO_PIN_14)

#elif defined(__ADSPSC584__)

/*
 * Push button 1 GPIO settings
 */

/* GPIO port to which push button 1 is connected to */
#define PUSH_BUTTON1_PORT           (ADI_GPIO_PORT_A)

/* GPIO pint to which push button 1 is connected to */
#define PUSH_BUTTON1_PINT           (ADI_GPIO_PIN_INTERRUPT_0)

/* GPIO pin within the port to which push button 1 is connected to */
#define PUSH_BUTTON1_PIN            (ADI_GPIO_PIN_15)

/* pin within the pint to which push button 1 is connected to */
#define PUSH_BUTTON1_PINT_PIN       (ADI_GPIO_PIN_31)

/* Label printed on the EZ-Kit */
#define PUSH_BUTTON1_LABEL          "PB1"

/*
 * Push button 2 GPIO settings
 */

/* GPIO port to which push button 2 is connected to */
#define PUSH_BUTTON2_PORT           (ADI_GPIO_PORT_B)

/* GPIO port to which push button 1 is connected to */
#define PUSH_BUTTON2_PINT           (ADI_GPIO_PIN_INTERRUPT_0)

/* GPIO pin within the port to which push button 1 is connected to */
#define PUSH_BUTTON2_PIN            (ADI_GPIO_PIN_0)

/* pin within the pint to which push button 1 is connected to */
#define PUSH_BUTTON2_PINT_PIN       (ADI_GPIO_PIN_0)

/* Label printed on the EZ-Kit */
#define PUSH_BUTTON2_LABEL          "PB2"


/*
 * LED 1 GPIO settings
 */

/* GPIO port to which LED 1 is connected to */
#define LED1_PORT                   (ADI_GPIO_PORT_E)

/* GPIO pin within the port to which LED 1 is connected to */
#define LED1_PIN                    (ADI_GPIO_PIN_1)


/*
 * LED 2 GPIO settings
 */

/* GPIO port to which LED 2 is connected to */
#define LED2_PORT                   (ADI_GPIO_PORT_E)

/* GPIO pin within the port to which LED 2 is connected to */
#define LED2_PIN                    (ADI_GPIO_PIN_2)


#elif defined(__ADSPSC573__)
/*
 * Push button 1 GPIO settings
 */

/* GPIO port to which push button 1 is connected to */
#define PUSH_BUTTON1_PORT           ADI_GPIO_PORT_C

/* GPIO pint to which push button 1 is connected to */
#define PUSH_BUTTON1_PINT           ADI_GPIO_PIN_INTERRUPT_1

/* GPIO pin within the port to which push button 1 is connected to */
#define PUSH_BUTTON1_PIN            ADI_GPIO_PIN_8

/* pin within the pint to which push button 1 is connected to */
#define PUSH_BUTTON1_PINT_PIN       (ADI_GPIO_PIN_8)
/* Label printed on the EZ-Kit */
#define PUSH_BUTTON1_LABEL          "PB1"

/*
 * Push button 2 GPIO settings
 */

/* GPIO port to which push button 2 is connected to */
#define PUSH_BUTTON2_PORT           ADI_GPIO_PORT_C

/* GPIO port to which push button 2 is connected to */
#define PUSH_BUTTON2_PINT           ADI_GPIO_PIN_INTERRUPT_1

/* GPIO pin within the port to which push button 2 is connected to */
#define PUSH_BUTTON2_PIN            ADI_GPIO_PIN_9

/* pin within the pint to which push button 1 is connected to */
#define PUSH_BUTTON2_PINT_PIN       (ADI_GPIO_PIN_9)
/* Label printed on the EZ-Kit */
#define PUSH_BUTTON2_LABEL          "PB2"

/*
 * LED 1 GPIO settings
 */

/* GPIO port to which LED 1 is connected to */
#define LED1_PORT                   ADI_GPIO_PORT_E

/* GPIO pin within the port to which LED 1 is connected to */
#define LED1_PIN                    ADI_GPIO_PIN_13

/*
 * LED 2 GPIO settings
 */

/* GPIO port to which LED 2 is connected to */
#define LED2_PORT                   ADI_GPIO_PORT_A

/* GPIO pin within the port to which LED 2 is connected to */
#define LED2_PIN                    ADI_GPIO_PIN_9
#endif

#endif /* _BUTTON_LED_GPIO_H_ */
