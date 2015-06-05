/*
#####################################################################################
#								In the name of ALLAH 								#
# Project : Direct to Phone															#
# Date : 2015/06/04																	#
# file : System.c 																	#
#																					#
#####################################################################################
*/

#include "stm32f4xx.h"
#include "types.h"
#include "SystemInitial.h"
#include "stdio.h"
#include "RTL.h"
#include "GeneralDefines.h" 

/******************************************************************************************************/
//Global Defines

/******************************************************************************************************/
//Global Variables

/******************************************************************************************************/
//Functions
void SetResetIO(GPIO_TypeDef *GpioPort, uint32 GpioPin, IOState Value){
	
	if (Value == enmReset)
		GpioPort -> ODR &= ~(1 << GpioPin);
	else 
		GpioPort -> ODR |= (1 << GpioPin);

}

/******************************************************************************************************/
void SystemConfiguration(void){
//Set Serial Interface Si3056 in MODE 0 as Master 
	SetResetIO(GPIOE, SI_M0, enmReset);
	SetResetIO(GPIOE, SI_M1, enmReset);	
	SetResetIO(GPIOE, SI_OFHK, enmSet);
	SetResetIO(GPIOE, SI_RESET, enmReset);	
	SetResetIO(GPIOE, SI_RESET, enmSet);
}














