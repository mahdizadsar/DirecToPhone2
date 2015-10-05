/*
#####################################################################################
#								In the name of ALLAH 								#
# Project : STM32F407 Test Project													#
# Date : 2013/11/18																	#
# file : SystemInitial.h															#
#																					#
#####################################################################################
*/
#include "stm32f4xx.h"
#include "types.h"
#include "GeneralDefines.h" 


void DmaConfig(	DMA_Stream_TypeDef *Dma, 		//Set DMA Stream Number Here
				uint32 PeripheralAddr, 			//Peripheral Address Specify Here
				uint32 Memory0Addr, 			//Memory 0 Address Specify Here
				uint32 Memory1Addr,				//Memory 1 Address Specify Here
				uint16 Size						//Size of DMA Beats Specify Here
);

void DmaEnable(	DMA_Stream_TypeDef *Dma, 			//Set DMA Stream Number Here
				boolean Enable						// Trur = Enable or False = Disable
);



uint16 SI3056ReadRegister(uint8 RegisterNumber);

void SetResetIO(GPIO_TypeDef *GpioPort, uint32 GpioPin, IOState Value);
void SystemConfiguration(void);
