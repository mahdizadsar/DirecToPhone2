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
#define UDPPORTNUMBER				5060
#define TCPPORTNUMBER				9000
#define UDPSIGNALINGPORT			5060

#define UDPMEDIAPORT				8000

#define REGISTRATION				0x04
#define KEEPALIVE					0x03
#define OFFHOOK						0xFE
#define ONHOOK						0xFF
#define CALL						0x05


#define MACLEN						6	
#define IPLEN						4
#define NAMELEN						20
/******************************************************************************************************/
//General Macros 

//Port E
#define SI_FC			10
#define SI_M1			11
#define SI_M0			12
#define SI_OFHK			13
#define SI_RESET		14

//Port D
#define DOUT			9
#define HT9032_PWRDOWN	10
#define HT9032_RING		11

#define  PrintDebug(...) {\
						USART3 -> BRR = 0x00000145;\
						delay(2);\
						printf(__VA_ARGS__);\
						delay(2);\
						USART3 -> BRR = 0x00007A12;\
						}


#endif 
						
						