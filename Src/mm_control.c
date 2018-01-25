#include <mm_control.h>
#include "cmsis_os.h"
#include <math.h>
#include <stdlib.h>
#include <usart.h>

int32_t delta[4] = {0};
int wAct[4] = {0};
int pulsCnt[4] = {0};
int mm_ref[4] = {0};
uint32_t MM_TimerPeriod = 0;
extern xQueueHandle mm_motorISRqueue[4];
extern xQueueHandle xQueueMotorSetpoint[4];

void mm_control_init(void)
{
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_OC_Start_IT(&htim4, TIM_CHANNEL_4);
	
	MM_TimerPeriod = (uint16_t)((HAL_RCC_GetPCLK1Freq() * 2)/((MM_TIMER_PRESCALER+1)*MM_TIMER_FREQUENCY));
	delta[0] = MM_TimerPeriod;
	delta[1] = MM_TimerPeriod;
	delta[2] = MM_TimerPeriod;
	delta[3] = MM_TimerPeriod;
}

void mm_control_motor1_set_compare(TIM_HandleTypeDef *htim, uint32_t period)
{
  /* Set the Capture Compare4 Register value */
  htim->Instance->CCR1 = period;
}

void mm_control_motor2_set_compare(TIM_HandleTypeDef *htim, uint32_t period)
{
  /* Set the Capture Compare4 Register value */
  htim->Instance->CCR2 = period;
}

void mm_control_motor3_set_compare(TIM_HandleTypeDef *htim, uint32_t period)
{
  /* Set the Capture Compare4 Register value */
  htim->Instance->CCR3 = period;
}

void mm_control_motor4_set_compare(TIM_HandleTypeDef *htim, uint32_t period)
{
  /* Set the Capture Compare4 Register value */
  htim->Instance->CCR4 = period;
}

int mm_motor_get_ref_position(int motor)
{
	return mm_ref[motor-1];
}

void mm_control_IRQHandler(TIM_HandleTypeDef *htim)
{
	static GPIO_PinState bitValue = GPIO_PIN_RESET;
	//static int32_t delta = 0;
	static int16_t direction[4] = {0};
	uint32_t tmp1 = 0, tmp2 = 0;
	MM_ISR_message_TypeDef mm_queue;
	BaseType_t xTaskWokenByReceive = pdFALSE;
	uint32_t curTim;
	
	//MOTOR1
	tmp1 = __HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC1);
  tmp2 = __HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC1);
	
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
		if(xQueueReceiveFromISR(mm_motorISRqueue[0], (void *)&mm_queue, &xTaskWokenByReceive) == pdTRUE)
		{
			delta[0] 		= mm_queue.delta;
			direction[0] = mm_queue.direction;
		}
			
		if(delta[0] != -1)
		{
			bitValue = (bitValue == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;

			HAL_GPIO_WritePin(MOTOR1_STEP_GPIO_Port, MOTOR1_STEP_Pin, bitValue);
				
			if(bitValue == GPIO_PIN_RESET)
			{
				if(direction[0] == 1)
				{
					pulsCnt[0] = pulsCnt[0] + 1;
				}
				else if (direction[0] == -1)
				{
					pulsCnt[0] = pulsCnt[0] - 1;
				}
			}
		}
			
		//Set direction pin properly
		if(direction[0] == 1)
		{
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_RESET);
		}
		else if(direction[0] == -1)
		{
			HAL_GPIO_WritePin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin, GPIO_PIN_SET);
		}
			
		//Set next OC ISR time
		curTim = __HAL_TIM_GET_COUNTER(htim);
		if (delta[0] != -1)
		{
			mm_control_motor1_set_compare(htim,(curTim+delta[0])%MM_TimerPeriod);
		}
		else
		{
			mm_control_motor1_set_compare(htim,(curTim+MM_TimerPeriod)%MM_TimerPeriod);
		}

		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC1);
	}
	
	//MOTOR2
	tmp1 = __HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC2);
  tmp2 = __HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC2);
	
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
		if(xQueueReceiveFromISR(mm_motorISRqueue[1], (void *)&mm_queue, &xTaskWokenByReceive) == pdTRUE)
		{
			delta[1] 		= mm_queue.delta;
			direction[1] = mm_queue.direction;
		}
			
		if(delta[1] != -1)
		{
			bitValue = (bitValue == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;

			HAL_GPIO_WritePin(MOTOR2_STEP_GPIO_Port, MOTOR2_STEP_Pin, bitValue);
				
			if(bitValue == GPIO_PIN_RESET)
			{
				if(direction[1] == 1)
				{
					pulsCnt[1] = pulsCnt[1] + 1;
				}
				else if (direction[1] == -1)
				{
					pulsCnt[1] = pulsCnt[1] - 1;
				}
			}
		}
			
		//Set direction pin properly
		if(direction[1] == 1)
		{
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_RESET);
		}
		else if(direction[1] == -1)
		{
			HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_SET);
		}
			
		//Set next OC ISR time
		curTim = __HAL_TIM_GET_COUNTER(htim);
		if (delta[1] != -1)
		{
			mm_control_motor2_set_compare(htim,(curTim+delta[1])%MM_TimerPeriod);
		}
		else
		{
			mm_control_motor2_set_compare(htim,(curTim+MM_TimerPeriod)%MM_TimerPeriod);
		}

		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC2);
	}
	
	//MOTOR3
	tmp1 = __HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3);
  tmp2 = __HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC3);
	
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
		if(xQueueReceiveFromISR(mm_motorISRqueue[2], (void *)&mm_queue, &xTaskWokenByReceive) == pdTRUE)
		{
			delta[2] 		= mm_queue.delta;
			direction[2] = mm_queue.direction;
		}
			
		if(delta[2] != -1)
		{
			bitValue = (bitValue == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;

			HAL_GPIO_WritePin(MOTOR3_STEP_GPIO_Port, MOTOR3_STEP_Pin, bitValue);
				
			if(bitValue == GPIO_PIN_RESET)
			{
				if(direction[2] == 1)
				{
					pulsCnt[2] = pulsCnt[2] + 1;
				}
				else if (direction[2] == -1)
				{
					pulsCnt[2] = pulsCnt[2] - 1;
				}
			}
		}
			
		//Set direction pin properly
		if(direction[2] == 1)
		{
			HAL_GPIO_WritePin(MOTOR3_DIR_GPIO_Port, MOTOR3_DIR_Pin, GPIO_PIN_RESET);
		}
		else if(direction[2] == -1)
		{
			HAL_GPIO_WritePin(MOTOR3_DIR_GPIO_Port, MOTOR3_DIR_Pin, GPIO_PIN_SET);
		}
			
		//Set next OC ISR time
		curTim = __HAL_TIM_GET_COUNTER(htim);
		if (delta[2] != -1)
		{
			mm_control_motor3_set_compare(htim,(curTim+delta[2])%MM_TimerPeriod);
		}
		else
		{
			mm_control_motor3_set_compare(htim,(curTim+MM_TimerPeriod)%MM_TimerPeriod);
		}

		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC3);
	}
	
	//MOTOR4
	tmp1 = __HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC4);
  tmp2 = __HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_CC4);
	
	if((tmp1 != RESET) && (tmp2 != RESET))
	{
		HAL_GPIO_TogglePin(LED_ORANGE_GPIO_Port, LED_ORANGE_Pin);
		if(xQueueReceiveFromISR(mm_motorISRqueue[3], (void *)&mm_queue, &xTaskWokenByReceive) == pdTRUE)
		{
			delta[3] 		= mm_queue.delta;
			direction[3] = mm_queue.direction;
		}
			
		if(delta[3] != -1)
		{
			bitValue = (bitValue == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;

			HAL_GPIO_WritePin(MOTOR4_STEP_GPIO_Port, MOTOR4_STEP_Pin, bitValue);
				
			if(bitValue == GPIO_PIN_RESET)
			{
				if(direction[3] == 1)
				{
					pulsCnt[3] = pulsCnt[3] + 1;
				}
				else if (direction[3] == -1)
				{
					pulsCnt[3] = pulsCnt[3] - 1;
				}
			}
		}
			
		//Set direction pin properly
		if(direction[3] == 1)
		{
			HAL_GPIO_WritePin(MOTOR4_DIR_GPIO_Port, MOTOR4_DIR_Pin, GPIO_PIN_RESET);
		}
		else if(direction[3] == -1)
		{
			HAL_GPIO_WritePin(MOTOR4_DIR_GPIO_Port, MOTOR4_DIR_Pin, GPIO_PIN_SET);
		}
			
		//Set next OC ISR time
		curTim = __HAL_TIM_GET_COUNTER(htim);
		if (delta[3] != -1)
		{
			mm_control_motor4_set_compare(htim,(curTim+delta[3])%MM_TimerPeriod);
		}
		else
		{
			mm_control_motor4_set_compare(htim,(curTim+MM_TimerPeriod)%MM_TimerPeriod);
		}

		__HAL_TIM_CLEAR_IT(htim, TIM_IT_CC4);
	}
		
	if( xTaskWokenByReceive != pdFALSE)
	{
			portYIELD_FROM_ISR( xTaskWokenByReceive );
	}
}

int sign(int value)
{
	if(value < 0)
	{
		return (-1);
	}
	else if (value > 0)
	{
		return (1);
	}
	else
	{
		return (0);
	}
}

int getPulsCnt(int motor_id)
{
	int ret = 0;
	HAL_NVIC_DisableIRQ(TIM4_IRQn);
	ret = pulsCnt[motor_id-1];
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	return ret;
}

void setPulsCnt(int count, int motor_id)
{
	HAL_NVIC_DisableIRQ(TIM4_IRQn);
	pulsCnt[motor_id-1] = count;
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void mm_control_algorithm(mmControl_TypeDef *params, int motor_id)
{
	int e;
	int wRef = 0;			//Output from proportinal regulator, before rate limiting [pps]
	int wNew = 0;			//New speed for controller 
	int rLim_l = params->acc_max / params->SamplingFrequency;
	int dLim_l = params->acc_max / params->SamplingFrequency;
	int wMax	= params->omega_max;	// pps	-> 300 rpm for 1600 cnt
	int delta_ = 0;						//Time delay between output compare ISR executions expresed in timer ticks, 
																// -1 value signals that the motor should not move
	int direction_ = 0;			//1 - forward, -1 - backward, 0 - stop
	int stepAct = 0;
	int ref = 0;
	MM_ISR_message_TypeDef msg;
	int32_t setpointMsg;
	
	//Checking if there is new setpoint available in the queue 
	if(xQueueReceive( xQueueMotorSetpoint[motor_id-1], (void *)&setpointMsg, (TickType_t) 0) == pdTRUE)
	{
		ref = setpointMsg;
	}
	
	//Get current step count
	stepAct = getPulsCnt(motor_id);
	//Control error signal
	e = ref - stepAct;
	
	// implement dead zone
	if (fabs(e) < params->dead_zone)
	{
		e = 0;
	}
	else if (e > 0)
	{
		e = e - params->dead_zone;
	}
	else
	{
		e = e + params->dead_zone;
	}
	
	wRef = e*params->P;
	
	//rate limiting 
	if((wRef-wAct[motor_id-1])>rLim_l)
	{
		wNew = wAct[motor_id-1]+rLim_l;
	}
	else if((wAct[motor_id-1] - wRef) > dLim_l)
	{
		wNew = wAct[motor_id-1]-dLim_l;
	}
	else
	{
		wNew = wRef;
	}
	
	//Saturation
	if(abs(wNew) > wMax)
	{
		if(sign(wNew) == -1)
		{
			wNew = -1*wMax;
		}
		else if(sign(wNew)==1)
		{
			wNew = wMax;
		}
	}
	
	//Calculate delay between steps 
	if(wNew == 0)
	{
		direction_ = 0;
		delta_ = -1;			// Signals ISR that motor needs to be stoped 
	}
	else
	{
		// period is 0.1 sec, so for 1 sec we need 10 * PERIOD ticks
		delta_ = (10 * MM_TimerPeriod/(2*abs(wNew)));	//2 because it has to turn the step pin on and off in real delta time
		if (delta_ > MM_TimerPeriod)
			delta_ = MM_TimerPeriod; // minimum speed > 0 is 10 pps
		direction_ = sign(wNew);
	}
	
	//Send to timer output compare ISR via correct motor queue 
	//Form a message
	msg.delta = delta_;
	msg.direction = direction_;
	//Add to Queue
	xQueueOverwrite(mm_motorISRqueue[motor_id-1],
		(void *)&msg);			
													
	//Set new compare on speed change for fast response
	if(wNew != wAct[motor_id-1])
	{
		if (motor_id == 1) mm_control_motor1_set_compare(&htim4,(__HAL_TIM_GET_COUNTER(&htim4)+delta_)%MM_TimerPeriod);
		else if (motor_id == 2) mm_control_motor2_set_compare(&htim4,(__HAL_TIM_GET_COUNTER(&htim4)+delta_)%MM_TimerPeriod);
		else if (motor_id == 3) mm_control_motor3_set_compare(&htim4,(__HAL_TIM_GET_COUNTER(&htim4)+delta_)%MM_TimerPeriod);
		else if (motor_id == 4) mm_control_motor4_set_compare(&htim4,(__HAL_TIM_GET_COUNTER(&htim4)+delta_)%MM_TimerPeriod);
	}
	
	//save new speed as actual speed for next step 
	wAct[motor_id-1] = wNew;
}
