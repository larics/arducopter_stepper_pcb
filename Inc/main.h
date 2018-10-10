/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define MOTOR4_DIR_Pin GPIO_PIN_0
#define MOTOR4_DIR_GPIO_Port GPIOC
#define MOTOR4_STEP_Pin GPIO_PIN_1
#define MOTOR4_STEP_GPIO_Port GPIOC
#define MOTOR3_DIR_Pin GPIO_PIN_2
#define MOTOR3_DIR_GPIO_Port GPIOC
#define MOTOR3_STEP_Pin GPIO_PIN_3
#define MOTOR3_STEP_GPIO_Port GPIOC
#define MOTOR1_DIR_Pin GPIO_PIN_1
#define MOTOR1_DIR_GPIO_Port GPIOA
#define MOTOR1_STEP_Pin GPIO_PIN_2
#define MOTOR1_STEP_GPIO_Port GPIOA
#define MOTOR2_DIR_Pin GPIO_PIN_3
#define MOTOR2_DIR_GPIO_Port GPIOA
#define MOTOR2_STEP_Pin GPIO_PIN_4
#define MOTOR2_STEP_GPIO_Port GPIOA
#define LED_BLUE_Pin GPIO_PIN_12
#define LED_BLUE_GPIO_Port GPIOB
#define LED_YELLOW_Pin GPIO_PIN_13
#define LED_YELLOW_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOB
#define LED_ORANGE_Pin GPIO_PIN_15
#define LED_ORANGE_GPIO_Port GPIOB
#define U6SEL_Pin GPIO_PIN_8
#define U6SEL_GPIO_Port GPIOC
#define MOTOR_MS1_Pin GPIO_PIN_10
#define MOTOR_MS1_GPIO_Port GPIOC
#define MOTOR_MS2_Pin GPIO_PIN_11
#define MOTOR_MS2_GPIO_Port GPIOC
#define MOTOR_MS3_Pin GPIO_PIN_12
#define MOTOR_MS3_GPIO_Port GPIOC

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define MM_TIMER_PRESCALER 279
#define MM_TIMER_FREQUENCY 10
#define MM_CONTROL_SAMPLING_FREQUENCY (uint32_t)100
#define BUFSIZE				20*20
#define BAUDRATE			115200
#define PACKSIZE			20					//size of data packet in bytes
#define SCU_ID				31
#define MM_CONTROL_P	14
#define MM_CONTROL_OMEGA_MAX 4000
#define MM_CONTROL_ACC_MAX 27500
#define MM_CONTROL_DEAD_ZONE 5
#define SW_VERSION_MAJOR 1
#define SW_VERSION_MINOR 0
#define HW_VERSION_MAJOR 1
#define HW_VERSION_MINOR 0
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
