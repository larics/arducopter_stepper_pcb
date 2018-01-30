/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include <mm_control.h>
#include <usart.h>
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId mm1ControlHandle;
osThreadId mm2ControlHandle;
osThreadId mm3ControlHandle;
osThreadId mm4ControlHandle;

xQueueHandle mm_motorISRqueue[4];
xQueueHandle xQueueMotorSetpoint[4];
xQueueHandle xQueueMotorSetup[4];

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void mmControlTask(void const * argument);
void vDecodeMsgTask(void *argument);
xTaskHandle xDecodeMsgTaskHandle;
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	
  /* definition and creation of mmControl */
  osThreadDef(MOTOR1_TASK, mmControlTask, osPriorityRealtime, 0, 512);
  mm1ControlHandle = osThreadCreate(osThread(MOTOR1_TASK), (void *)1);
	
  /* definition and creation of mmControl */
  osThreadDef(MOTOR2_TASK, mmControlTask, osPriorityRealtime, 0, 512);
  mm2ControlHandle = osThreadCreate(osThread(MOTOR2_TASK), (void *)2);
	
  /* definition and creation of mmControl */
  osThreadDef(MOTOR3_TASK, mmControlTask, osPriorityRealtime, 0, 512);
  mm3ControlHandle = osThreadCreate(osThread(MOTOR3_TASK), (void *)3);
	
  /* definition and creation of mmControl */
  osThreadDef(MOTOR4_TASK, mmControlTask, osPriorityRealtime, 0, 512);
  mm4ControlHandle = osThreadCreate(osThread(MOTOR4_TASK), (void *)4);
	
	/* definition and creation of mmControl */
	xTaskCreate(
		vDecodeMsgTask,
		(const char*)"MSG_DECODE_TASK",
		configMINIMAL_STACK_SIZE,
		NULL,
		2,		
		&xDecodeMsgTaskHandle); 
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	mm_motorISRqueue[0] = xQueueCreate(1, sizeof(MM_ISR_message_TypeDef));
	mm_motorISRqueue[1] = xQueueCreate(1, sizeof(MM_ISR_message_TypeDef));
	mm_motorISRqueue[2] = xQueueCreate(1, sizeof(MM_ISR_message_TypeDef));
	mm_motorISRqueue[3] = xQueueCreate(1, sizeof(MM_ISR_message_TypeDef));
	
	xQueueMotorSetpoint[0] = xQueueCreate(1, sizeof(int32_t));
	xQueueMotorSetpoint[1] = xQueueCreate(1, sizeof(int32_t));
	xQueueMotorSetpoint[2] = xQueueCreate(1, sizeof(int32_t));
	xQueueMotorSetpoint[3] = xQueueCreate(1, sizeof(int32_t));
  
	xQueueMotorSetup[0] = xQueueCreate(1, sizeof(motor_setup_t));
	xQueueMotorSetup[1] = xQueueCreate(1, sizeof(motor_setup_t));
	xQueueMotorSetup[2] = xQueueCreate(1, sizeof(motor_setup_t));
	xQueueMotorSetup[3] = xQueueCreate(1, sizeof(motor_setup_t));		
	/* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */

/* mmControlTask function */
void mmControlTask(void const * argument)
{
  /* USER CODE BEGIN mmControlTask */
  mmControl_TypeDef mm_motor_parameters;
	uint32_t motorNumber = (uint32_t)argument;
	motor_setup_t setupMsg;
	
	mm_motor_parameters.P = 2;
	mm_motor_parameters.acc_max = 1000000;
	mm_motor_parameters.dead_zone = 0;
	mm_motor_parameters.omega_max = 1200000;
	mm_motor_parameters.SamplingFrequency = MM_CONTROL_SAMPLING_FREQUENCY;

  /* Infinite loop */
  for(;;)
  {
		if(xQueueReceive(xQueueMotorSetup[motorNumber-1],(void *)&setupMsg,(TickType_t) 0) == pdTRUE)
		{
			//Refresh controler parameters
			mm_motor_parameters.P = setupMsg.P;
			mm_motor_parameters.omega_max = setupMsg.wMax;
			mm_motor_parameters.acc_max = setupMsg.rLim;
			mm_motor_parameters.dead_zone = setupMsg.dLim;;
		}
		mm_control_algorithm(&mm_motor_parameters, motorNumber);
		osDelay(1000.0/MM_CONTROL_SAMPLING_FREQUENCY);
  }
}

void vDecodeMsgTask(void *argument)
{
	union Data
		{
			motor_control_t broj;
			char str[20];
		} data;
		
	uint16_t i;
		
	while(1)
	{
		//If there is notification do some work  
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY  );			
		
		//Fill up data structure 
		for(i=0; i<20; i++)
		{
			USART1_Dequeue(&data.str[i]);
		}
		//Padding bytes set to zero 
		data.str[19] = 0;
		data.str[18] = 0;
		data.str[17] = 0;
		
		//From cmd data field decide where to send the data

		switch((char)data.broj.cmd)
		{
			case('C'):
			{
				//Send to motor setpoint queues
				int32_t ref = data.broj.moto1;
				//Send to appropriate setpoint queues. Queues hold only 1 element,
				// and xQueueOverwrite allways inserts the last received value to 
				// queue even if it's full (overwrite)
				xQueueOverwrite(xQueueMotorSetpoint[0],
												(void *)&ref);
												
				ref = data.broj.moto2;
				xQueueOverwrite(xQueueMotorSetpoint[1],
												(void *)&ref);
				
				ref = data.broj.moto3;
				xQueueOverwrite(xQueueMotorSetpoint[2],
												(void *)&ref);
				
				ref = data.broj.moto4;
				xQueueOverwrite(xQueueMotorSetpoint[3],
												(void *)&ref);
				
				//In final version send to all setpoint queues
				break;
										
			}
			case('S'):
			{
				// Setup msg for all motor controllers 
				
					//Form a message 
					motor_setup_t setupCmd;
						setupCmd.P 		= data.broj.moto1;
						setupCmd.wMax = data.broj.moto2;
						setupCmd.rLim = data.broj.moto3;
						setupCmd.dLim = data.broj.moto4;
					//Add to queue without blocking 
					xQueueOverwrite( xQueueMotorSetup[0],
														(void *)&setupCmd);
					xQueueOverwrite( xQueueMotorSetup[1],
														(void *)&setupCmd);
					xQueueOverwrite( xQueueMotorSetup[2],
														(void *)&setupCmd);
					xQueueOverwrite( xQueueMotorSetup[3],
														(void *)&setupCmd);
				break;
			}
			/*case('M'):
			{
				//Set global shared pule counter variables in critical section
				NVIC_DisableIRQ(TIM4_IRQn);
					pulsCnt1 = data.broj.moto1;
					pulsCnt2 = data.broj.moto2;
					pulsCnt3 = data.broj.moto3;
					pulsCnt4 = data.broj.moto4;
				NVIC_EnableIRQ(TIM4_IRQn);
			}*/
			default:
				break;
		}
				
	}
}
  /* USER CODE END mmControlTask */
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
