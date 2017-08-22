/* gpio.h */
#ifndef GPIO_H
#define GPIO_H
#include <stm32f4xx.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
// GPIO RCC Peripheral defines 
#define MOTOR1_RCC_GPIOx 				RCC_AHB1Periph_GPIOA
#define MOTOR2_RCC_GPIOx				RCC_AHB1Periph_GPIOA
#define MOTOR3_RCC_GPIOx				RCC_AHB1Periph_GPIOC
#define MOTOR4_RCC_GPIOx				RCC_AHB1Periph_GPIOC
#define MS_RCC_GPIOx						RCC_AHB1Periph_GPIOC

// GPIO Peripheral defines
#define MOTOR1_GPIOx						GPIOA
#define MOTOR2_GPIOx						GPIOA
	//motor 3 is a little special due to soldering errors
#define MOTOR3_GPIOx						GPIOC
#define MOTOR4_GPIOx						GPIOC
#define MS_GPIOx								GPIOC

// Pin numbers for motors
	//Motor 1 
#define MOTOR1_DIR							GPIO_Pin_1
#define MOTOR1_STEP							GPIO_Pin_2
#define MOTOR1_MS1							GPIO_Pin_10
#define MOTOR1_MS2							GPIO_Pin_11
#define MOTOR1_MS3							GPIO_Pin_12

	//Motor 2
#define MOTOR2_DIR							GPIO_Pin_3
#define MOTOR2_STEP							GPIO_Pin_4
#define MOTOR2_MS1							GPIO_Pin_10
#define MOTOR2_MS2							GPIO_Pin_11
#define MOTOR2_MS3							GPIO_Pin_12

	//Motor 3
#define MOTOR3_DIR							GPIO_Pin_2
#define MOTOR3_STEP							GPIO_Pin_3
#define MOTOR3_MS1							GPIO_Pin_10
#define MOTOR3_MS2							GPIO_Pin_11
#define MOTOR3_MS3							GPIO_Pin_12

	//Motor 4
#define MOTOR4_DIR							GPIO_Pin_0
#define MOTOR4_STEP							GPIO_Pin_1
#define MOTOR4_MS1							GPIO_Pin_10
#define MOTOR4_MS2							GPIO_Pin_11
#define MOTOR4_MS3							GPIO_Pin_12

#define LED_BLUE								GPIO_Pin_12
#define LED_YELLOW							GPIO_Pin_13
#define LED_RED									GPIO_Pin_14
#define LED_ORANGE							GPIO_Pin_15

#define LED_GPIOx								GPIOB
#define LED_RCC_GPIOx						RCC_AHB1Periph_GPIOB


void gpio_init(void);

#endif
