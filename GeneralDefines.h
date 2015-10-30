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
//General Enumeration Type Defines 
typedef enum {
	enmOnHook 	= 0,
	enmOffHook 	= 1,
	enmBusy		= 2
} enmDeviceState_t;

//--------------------------------------

typedef enum {
	enmReset = 0,
	enmSet	 = 1
}IOState;	
	
/******************************************************************************************************/
//General Structure Type Defines 
typedef struct {
	uint8 	NameLen;
	uint8 	Name[20];
	uint8 	MAC[6];
	uint8	ID;
	uint8 	IP[4];
	uint8	Status;
	uint16 	Port; 
}SpRecord_t;

//--------------------------------------

typedef struct {
	uint8 	ID;
	uint8 	Command;
	uint16 	Len;
	uint8 	Data[512];
}ReplyPacket_t;

//--------------------------------------

typedef struct {
	uint8 	ID;
	uint8 	Command;
	uint16 	Len;
	uint8 	Data[26];
	uint16 	CheckSum;
	uint8 	IP[4];
	uint16  Port;
}ReceivePacket_t;

/******************************************************************************************************/
//General Defines
//Ports
#define UDP_MEDIA_PORT				8000
#define UDP_CTRL_PORT				5060

//Commands
#define CHECKSUM_ERROR				0x00
#define	CMD_DISCOVERY				0x80
#define	CMD_CREATION_ACCOUNT		0x81
#define CMD_DELETE_ACCOUNT			0x82
#define	CMD_SPS_STATUS				0x83
#define	CMD_CALL					0x84
#define	CMD_ANSWER					0x85
#define	CMD_HANG_UP					0x86
#define	CMD_INTERCOM				0x87

#define CMD_STATUS					0x40
#define CMD_RING					0x41

#define ACK							0x50
#define NACK						0x51

#define D2P_ID						0xAA

#define DISCOVERY_REPLY_LEN			0x04
#define ACK_REPLY_LEN				0x00



#define PACKET_OVERHEAD				0x06
#define MACLEN						6	
#define NAMELEN						20

//DTMF len for 50ms Duration
#define DTMF_NUM_LEN				800

//Port E
#define SI_FC						10
#define SI_M1						11
#define SI_M0						12
#define SI_OFHK						13
#define SI_RESET					14
			
//Port D			
#define DOUT						9
#define HT9032_PWRDOWN				10
#define HT9032_RING					11


/******************************************************************************************************/
//General Macros 
#define  PrintDebug(...) {\
						USART3 -> BRR = 0x00000145;\
						delay(2);\
						printf(__VA_ARGS__);\
						delay(2);\
						USART3 -> BRR = 0x00007A12;\
						}


#endif
