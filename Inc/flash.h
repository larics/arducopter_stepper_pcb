/* flash.h */

#ifndef FLASH_H
#define FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <mm_control.h>

typedef union _data {
  float f;
  char  s[4];
} float2char;


/* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0    			((uint32_t)0x08000000)
#define ADDR_FLASH_SECTOR_1   		  ((uint32_t)0x08004000)
#define ADDR_FLASH_SECTOR_2   		  ((uint32_t)0x08008000)
#define ADDR_FLASH_SECTOR_3   		  ((uint32_t)0x0800C000)
#define ADDR_FLASH_SECTOR_4   		  ((uint32_t)0x08010000)
#define ADDR_FLASH_SECTOR_5   		  ((uint32_t)0x08020000)
#define ADDR_FLASH_SECTOR_6   		  ((uint32_t)0x08040000)
#define ADDR_FLASH_SECTOR_7   		  ((uint32_t)0x08060000)
#define ADDR_FLASH_SECTOR_8   		  ((uint32_t)0x08080000)
#define ADDR_FLASH_SECTOR_9				  ((uint32_t)0x080A0000)
#define ADDR_FLASH_SECTOR_10   			((uint32_t)0x080C0000)
#define ADDR_FLASH_SECTOR_11    		((uint32_t)0x080E0000)

#define FLASH_USER_PARAMS_START_ADDR  			ADDR_FLASH_SECTOR_11
#define FLASH_USER_PARAMS_END_ADDR					ADDR_FLASH_SECTOR_11

#define MM_CONTROL_P_ADDR																			(FLASH_USER_PARAMS_START_ADDR)
#define MM_CONTROL_SAMPLING_FREQUENCY_ADDR										(FLASH_USER_PARAMS_START_ADDR+0x04)
#define MM_CONTROL_OMEGA_MAX_ADDR															(FLASH_USER_PARAMS_START_ADDR+0x08)
#define MM_CONTROL_ACC_MAX_ADDR																(FLASH_USER_PARAMS_START_ADDR+0x0C)
#define MM_CONTROL_DEAD_ZONE_ADDR															(FLASH_USER_PARAMS_START_ADDR+0x10)


HAL_StatusTypeDef writeParams2Flash(mmControl_TypeDef *parameters);
HAL_StatusTypeDef readParametersFromFlash(mmControl_TypeDef *parameters);
HAL_StatusTypeDef readDefaultParameters(mmControl_TypeDef *parameters);

uint8_t get_scu_id(void);


#ifdef __cplusplus
}
#endif

#endif
