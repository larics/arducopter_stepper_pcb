/* gpio.c */
#include <gpio.h>

void gpio_init(void)
{
		GPIO_InitTypeDef GPIOStruct;
		GPIO_StructInit(&GPIOStruct);
		//Turn on clock for peripheral 
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
		//Pin configuration 
		GPIOStruct.GPIO_Pin		= GPIO_Pin_12;
		GPIOStruct.GPIO_Mode 	= GPIO_Mode_OUT;
		GPIOStruct.GPIO_OType = GPIO_OType_PP; 
		GPIOStruct.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
		GPIOStruct.GPIO_Speed	= GPIO_Speed_50MHz;
	//GPIO_Init(GPIOD, &GPIOStruct);
	//Green LED
	//GPIO_WriteBit(GPIOD, GPIO_Pin_12, Bit_RESET);
	//Blue LED
	//GPIOStruct.GPIO_Pin    	= GPIO_Pin_15;
	//GPIO_Init(GPIOD,&GPIOStruct);
	//Orange LED
	//GPIOStruct.GPIO_Pin    	= GPIO_Pin_13;
	//GPIO_Init(GPIOD,&GPIOStruct);
	
	RCC_AHB1PeriphClockCmd(MOTOR1_RCC_GPIOx, ENABLE);
		//Dir pin
		GPIOStruct.GPIO_Pin 	= MOTOR1_DIR;
		GPIO_Init(MOTOR1_GPIOx, &GPIOStruct);
		//Step pin
		GPIOStruct.GPIO_Pin 	= MOTOR1_STEP;
		GPIO_Init(MOTOR1_GPIOx, &GPIOStruct);		
		//Reset dir pin
		GPIO_WriteBit(MOTOR1_GPIOx, MOTOR1_DIR, Bit_RESET);

		
	//Motor 2 pin init 
	RCC_AHB1PeriphClockCmd(MOTOR2_RCC_GPIOx, ENABLE);
		//Dir pin motor2
		GPIOStruct.GPIO_Pin = MOTOR2_DIR;
		GPIO_Init(MOTOR2_GPIOx, &GPIOStruct);
		//Step pin motor2
		GPIOStruct.GPIO_Pin = MOTOR2_STEP;
		GPIO_Init(MOTOR2_GPIOx, &GPIOStruct);		
		//Reset dir pin
		GPIO_WriteBit(MOTOR2_GPIOx, MOTOR2_DIR, Bit_RESET);

	//Motor 3 pin init 
	RCC_AHB1PeriphClockCmd(MOTOR3_RCC_GPIOx, ENABLE);		//Motor 3 uses pins from GPIOA, GPIOC and GPIOD periph
		//Dir pin motor3
		GPIOStruct.GPIO_Pin = MOTOR3_DIR;
		GPIO_Init(MOTOR3_GPIOx, &GPIOStruct);
		//Step pin motor3
		GPIOStruct.GPIO_Pin = MOTOR3_STEP;
		GPIO_Init(MOTOR3_GPIOx, &GPIOStruct);	
		//Reset dir pin
		GPIO_WriteBit(MOTOR3_GPIOx, MOTOR3_DIR, Bit_RESET);
	
	//Motor 4 pin init 
	RCC_AHB1PeriphClockCmd(MOTOR4_RCC_GPIOx, ENABLE);
		//Dir pin motor4
		GPIOStruct.GPIO_Pin = MOTOR4_DIR;
		GPIO_Init(MOTOR4_GPIOx, &GPIOStruct);
		//Step pin motor4
		GPIOStruct.GPIO_Pin = MOTOR4_STEP;
		GPIO_Init(MOTOR4_GPIOx, &GPIOStruct);
		//Reset dir pin
		GPIO_WriteBit(MOTOR4_GPIOx, MOTOR4_DIR, Bit_RESET);


	//MS pin init
	RCC_AHB1PeriphClockCmd(MS_RCC_GPIOx, ENABLE);
		//Init MS pins
		GPIOStruct.GPIO_Pin = MOTOR1_MS1;
		GPIO_Init(MS_GPIOx, &GPIOStruct);
		GPIOStruct.GPIO_Pin = MOTOR1_MS2;
		GPIO_Init(MS_GPIOx, &GPIOStruct);
		GPIOStruct.GPIO_Pin = MOTOR1_MS2;
		GPIO_Init(MS_GPIOx, &GPIOStruct);
		//Set MS pins to halfstep mode
		GPIO_WriteBit(MS_GPIOx, MOTOR1_MS1, Bit_SET);
		GPIO_WriteBit(MS_GPIOx, MOTOR1_MS2, Bit_RESET);
		GPIO_WriteBit(MS_GPIOx, MOTOR1_MS3, Bit_RESET);
		
		//LED setup 
		RCC_AHB1PeriphClockCmd(LED_RCC_GPIOx, ENABLE);
		GPIOStruct.GPIO_Pin = LED_BLUE;
		GPIO_Init(LED_GPIOx, &GPIOStruct);
		GPIOStruct.GPIO_Pin = LED_YELLOW;
		GPIO_Init(LED_GPIOx, &GPIOStruct);
		GPIOStruct.GPIO_Pin = LED_RED;
		GPIO_Init(LED_GPIOx, &GPIOStruct);
		GPIOStruct.GPIO_Pin = LED_ORANGE;
		GPIO_Init(LED_GPIOx, &GPIOStruct);
		
		//Turn on LEDs
		GPIO_WriteBit(LED_GPIOx, LED_BLUE, Bit_SET);
		GPIO_WriteBit(LED_GPIOx, LED_YELLOW, Bit_SET);
		GPIO_WriteBit(LED_GPIOx, LED_RED, Bit_SET);
		GPIO_WriteBit(LED_GPIOx, LED_ORANGE, Bit_SET);
}

