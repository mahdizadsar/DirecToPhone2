/*
#####################################################################################
#								In the name of ALLAH 								#
# Project : Direct to Phone															#
# Date : 2015/06/04																	#
# file : GeneralDeifnes.h															#
#																					#
#####################################################################################
*/

#include "stm32f4xx.h"
#include "types.h"


#ifndef __GENERALDEFINES__
#define __GENERALDEFINES__

/******************************************************************************************************/
typedef enum enmDeviceState_t{
	enmOnHook 	= 0,
	enmOffHook 	= 1,
	enmBusy		= 2
} enmDeviceState_t;

/******************************************************************************************************/
//General Type Defines 
typedef enum IOState {
	enmReset = 0,
	enmSet	 = 1
}IOState;	
		
/******************************************************************************************************/
//General Defines
#define UDPPORTNUMBER				8000
#define TCPPORTNUMBER				9000

/******************************************************************************************************/
//General Macros 

//Port E
#define SI_FC			10
#define SI_M1			11
#define SI_M0			12
#define SI_OFHK			13
#define SI_RESET		14

//Port D
#define HT9032_PWRDOWN	10
#define HT9032_RING		11


#endif 