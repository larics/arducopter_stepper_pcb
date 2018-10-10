#ifndef MM_CONTROL_H
#define MM_CONTROL_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "tim.h"

typedef struct
{
	float P;
	uint32_t SamplingFrequency;
	uint32_t omega_max;
	uint32_t acc_max;
	int dead_zone;
} mmControl_TypeDef;

typedef struct
{
	int32_t delta;
	int16_t	 direction;
} MM_ISR_message_TypeDef;

int sign(int value);
int mm_motor_get_ref_position(int motor);
void mm_control_init(void);
void setPulsCnt(int count, int motor_id);
int getPulsCnt(int motor_id);
void mm_control_IRQHandler(TIM_HandleTypeDef *htim);
void mm_control_algorithm(mmControl_TypeDef *params, int motor_id);
void mm_control_motor1_set_compare(TIM_HandleTypeDef *htim, uint32_t period);
void mm_control_motor2_set_compare(TIM_HandleTypeDef *htim, uint32_t period);
void mm_control_motor3_set_compare(TIM_HandleTypeDef *htim, uint32_t period);
void mm_control_motor4_set_compare(TIM_HandleTypeDef *htim, uint32_t period);

#ifdef __cplusplus
}
#endif
#endif
