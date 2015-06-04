/*
#####################################################################################
#								In the name of ALLAH 								#
# Project : STM32F407 Test Project													#
# Date : 2014/05/07																	#
# file : STM32F4EmbeddedFlash.c 													#
#																					#
#####################################################################################
*/

#include "stm32f4xx.h"
#include "types.h"
#include "STM32F4EmbeddedFlash.h"

/******************************************************************************************************/

boolean FlashCRLock(boolean Lock){
	if (Lock == False){
		FLASH -> KEYR = KEY1;
		FLASH -> KEYR = KEY2;
		if (!(FLASH -> CR & FLASH_CR_LOCK)) 
			return True;
		else
			return False;
	}
	else{
		while(FLASH -> SR & FLASH_SR_BSY);
		FLASH -> CR |= FLASH_CR_LOCK;
		if (FLASH -> CR & FLASH_CR_LOCK) 
			return True;
		else 
			return False;
	}
}

/******************************************************************************************************/

boolean FlashOptionByteCRLock(boolean Lock){
	if (Lock == False){
		FLASH -> OPTKEYR = OPTKEY1;
		FLASH -> OPTKEYR = OPTKEY2;
		if (!(FLASH -> OPTCR & FLASH_OPTCR_OPTLOCK)) 
			return True;
		else
			return False;
	}
	else{
		while(FLASH -> SR & FLASH_SR_BSY);
		FLASH -> OPTCR |= FLASH_OPTCR_OPTLOCK; 
		if (FLASH -> OPTCR & FLASH_OPTCR_OPTLOCK) 
			return True;
		else
			return False;	
	}
}

/******************************************************************************************************/
//Write x32 in Embedded Flash
boolean FlashWrite(uint32 *Data, uint32 *Address, uint32 Size){
	uint32 i;
	
	while(FLASH -> SR & FLASH_SR_BSY);
	FLASH -> CR |= FLASH_CR_PG;
	for (i = 0 ; i < Size ; i++)
		Address[i] = Data[i];
	
	while(FLASH -> SR & FLASH_SR_BSY);
	FLASH -> CR &= (uint32)~(FLASH_CR_PG);
	if (FLASH -> SR & (FLASH_SR_PGSERR | FLASH_SR_PGPERR | FLASH_SR_PGAERR |FLASH_SR_WRPERR |FLASH_SR_SOP)) return False;
	return True;
}

/******************************************************************************************************/

boolean FlashSectoreErase(SectorNumber){
	while(FLASH -> SR & FLASH_SR_BSY);
	FLASH -> CR |= FLASH_CR_SER | ((SectorNumber << 3) & FLASH_CR_SNB);
	FLASH -> CR |= FLASH_CR_STRT;
	while(FLASH -> SR & FLASH_SR_BSY);
	FLASH -> CR &= (uint32)~(FLASH_CR_SER);
	if (FLASH -> SR & (FLASH_SR_WRPERR | FLASH_SR_SOP)) return False;
	return True;
}

/******************************************************************************************************/

boolean FlashMassErase(void){
	while(FLASH -> SR & FLASH_SR_BSY);
	FLASH -> CR |= FLASH_CR_MER;
	FLASH -> CR |= FLASH_CR_STRT;
	while(FLASH -> SR & FLASH_SR_BSY);
	FLASH -> CR &= (uint32)~(FLASH_CR_MER);
	if (FLASH -> SR & (FLASH_SR_WRPERR | FLASH_SR_SOP)) return False;
	return True;
}

/******************************************************************************************************/

boolean ReadProtection(uint8 Level){
	
}
