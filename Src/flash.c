/* flash.c */

#include <flash.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <mm_control.h>
#include <math.h>

static uint32_t GetSector(uint32_t Address);
HAL_StatusTypeDef writeFloat2Flash(float param, uint32_t Address);
mmControl_TypeDef scu_parameters;

HAL_StatusTypeDef readParametersFromFlash(mmControl_TypeDef *parameters)
{	
	if (isnan(*(float*)(MM_CONTROL_P_ADDR))) return HAL_ERROR;
	else if (*(uint32_t*)(MM_CONTROL_SAMPLING_FREQUENCY_ADDR) == 0xFFFFFFFF ) return HAL_ERROR;
	else if (*(uint32_t*)(MM_CONTROL_OMEGA_MAX_ADDR) == 0xFFFFFFFF ) return HAL_ERROR;
	else if (*(uint32_t*)(MM_CONTROL_ACC_MAX_ADDR) == 0xFFFFFFFF ) return HAL_ERROR;
	else if (*(int*)(MM_CONTROL_DEAD_ZONE_ADDR) == 0xFFFFFFFF ) return HAL_ERROR;

	parameters->P = *(float*)MM_CONTROL_P_ADDR;
	parameters->omega_max = *(uint32_t*)MM_CONTROL_OMEGA_MAX_ADDR;
	parameters->SamplingFrequency = *(uint32_t*)MM_CONTROL_SAMPLING_FREQUENCY_ADDR;
	parameters->acc_max = *(uint32_t*)MM_CONTROL_ACC_MAX_ADDR;
	parameters->dead_zone = *(int*)MM_CONTROL_DEAD_ZONE_ADDR;

	return HAL_OK;
}

uint8_t get_scu_id(void)
{
	uint8_t id;
	
	id = SCU_ID;
	
	return id;
}

HAL_StatusTypeDef readDefaultParameters(mmControl_TypeDef *parameters)
{
	parameters->P = MM_CONTROL_P;
	parameters->omega_max = MM_CONTROL_OMEGA_MAX;
	parameters->SamplingFrequency = MM_CONTROL_SAMPLING_FREQUENCY;
	parameters->acc_max = MM_CONTROL_ACC_MAX;
	parameters->dead_zone = MM_CONTROL_DEAD_ZONE;

	return HAL_OK;
}

HAL_StatusTypeDef writeFloat2Flash(float param, uint32_t Address)
{
	float2char floatData;
	uint8_t count;
	
	floatData.f = param;
	for (count = 0; count<4; count ++)
	{
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (Address + count), floatData.s[count])!=HAL_OK)
		{
			// Error occurred while writing to flash
			return HAL_ERROR;
		}
	}
	
	return HAL_OK;
}

HAL_StatusTypeDef writeParams2Flash(mmControl_TypeDef *parameters)
{		
	/* Unlock the Flash to enable the flash control register access *************/ 
  HAL_FLASH_Unlock();
	
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	FLASH_Erase_Sector(GetSector(FLASH_USER_PARAMS_START_ADDR), VOLTAGE_RANGE_3);
	
	//write moving mass control sampling freq param to flash
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MM_CONTROL_SAMPLING_FREQUENCY_ADDR, parameters->SamplingFrequency)!=HAL_OK) return HAL_ERROR;
	//write max angular velocity of moving mass param to flash
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MM_CONTROL_OMEGA_MAX_ADDR, parameters->omega_max)!=HAL_OK) return HAL_ERROR;
	//write moving mass controler gain param to flash
	if (writeFloat2Flash(parameters->P, MM_CONTROL_P_ADDR) != HAL_OK) return HAL_ERROR;
	//write moving mass contoler max acc param to flash
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MM_CONTROL_ACC_MAX_ADDR, parameters->acc_max)!=HAL_OK) return HAL_ERROR;
	
	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MM_CONTROL_DEAD_ZONE, parameters->dead_zone)!=HAL_OK) return HAL_ERROR;
	
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock(); 
	
	return HAL_OK;
}

static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;
  
  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;  
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;  
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;  
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;  
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;  
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;  
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;  
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;  
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;  
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;  
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;  
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}
