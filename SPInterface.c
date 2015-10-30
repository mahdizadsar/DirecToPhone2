/*
#####################################################################################
#								In the name of ALLAH 								#
# Project : Direct to Phone															#
# Date : 2015/10/11																	#
# file : SpInterface.c 																	#
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
//External Variables
extern ReceivePacket_t ReceivePacket;

/******************************************************************************************************/

boolean UdpCommandParser(void){
	uint16 UdpCtrlPacketCheckSum = 0;
	uint16 i;

	if (ReceivePacket.Len > 0xFF){
		PrintDebug("\nError : Packcet Len Too Long");
		ReceivePacket.Command = 0x00;
	}
	
	//PrintDebug("\n");
	//for (i = 0 ; i < (ReceivePacket.Len + 4) ; i++)
	//	PrintDebug("0x%02X ",((uint8*)&ReceivePacket)[i]);
	
	for (i = 0 ; i < (ReceivePacket.Len + 4) ; i++)
		UdpCtrlPacketCheckSum += ((uint8*)&ReceivePacket)[i];
	
	if (UdpCtrlPacketCheckSum != ReceivePacket.CheckSum){
		PrintDebug("\nPacket Checksum Error");
		ReceivePacket.Command = 0x00;
	}
		
	
	switch(ReceivePacket.Command){
		case CMD_DISCOVERY:
			DiscoveryRoutine();
			//RegistrationFunction();
			break;
		case CMD_CREATION_ACCOUNT:
			CreationAccountRoutine();
			//KeepAliveFunction();
			break;
		case CMD_SPS_STATUS:
			//SpsStatusRoutine();
			//CallFunction();
			break;
		case CMD_CALL:
			CallRoutine();
			//OffHook();
			break;
		case CMD_ANSWER:
			AnswerRoutine();
			//OnHook();
			break;
		case CMD_HANG_UP:
			HangUpRoutine();
			break;
		case CMD_INTERCOM:
			//IntercomRoutine();
			break;
		case CHECKSUM_ERROR:
			//ErrorRoutine();
		default: 
			//GeneralErrorRoutine();
			break;					
		
	}
}