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
//General Type Defines 
typedef enum IOState {
	enmReset = 0,
	enmSet	 = 1
}IOState;	
				

/******************************************************************************************************/
//General Macros 

#define SI_FC			10
#define SI_M1			11
#define SI_M0			12
#define SI_RESET		14



#endif 