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
#include "Net_Config.h"

void SI3056WriteRegister(uint8 ,uint8);
/******************************************************************************************************/
//Global Defines

/******************************************************************************************************/
//Global Variables
uint8 			*BufferPtr;
uint16 			UdpCounter = 0;

ReplyPacket_t 	*ReplyPacket;

/******************************************************************************************************/
//Global Constant

/******************************************************************************************************/
//External Variables
extern uint8 				UdpCtrlSoc;				//UDP Signaling Handler
extern uint8 				UdpMediaSoc;			//UDP Media Handler
extern uint8 				UdpMediaRecieved;
extern LOCALM 				localm[];
extern ReceivePacket_t 		ReceivePacket;
extern uint8				DtmfCode[10][1600];		
extern enmDeviceState_n 	DeviceState;
extern uint8 				NumberOfSPs;
extern uint8 				SPtoSIMediaBuffer[128];
extern uint8 				SItoSPMediaBuffer[128];

extern SpRecord_t 			SpRecord[20];
extern SpRecord_ft			SpRecordFlash[20];



/******************************************************************************************************/
//Functions
void ReadSPsRecord(void){
	uint8 i;
	
	NumberOfSPs = 0;
	
	for (i = 0 ; i < 0xFF ; i++){
		if (SpRecordFlash[i].ID == 0xFF)	break;
		memcpy((uint8*)&SpRecord[i], (uint8*)&SpRecordFlash[i], sizeof(SpRecord_ft));
		NumberOfSPs++;
	}
}
/******************************************************************************************************/
void SetResetIO(GPIO_TypeDef *GpioPort, uint32 GpioPin, IOState_n Value){
	
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
	SetResetIO(GPIOE, SI_FC, enmReset);
	SetResetIO(GPIOE, SI_RESET, enmReset);	
	SetResetIO(GPIOE, SI_RESET, enmSet);
}

/******************************************************************************************************/

uint16 SI3056ReadRegister(uint8 RegisterNumber){
	
	uint16 DataInput;
	
 	while (!(SPI1 -> SR & SPI_SR_RXNE));
 	DataInput = SPI1 -> DR;
	
	while (!(SPI1 -> SR & SPI_SR_TXE));
	SPI1 -> DR = 0x0001;
	
	while (!(SPI1 -> SR & SPI_SR_RXNE));	
	DataInput = SPI1 -> DR;
	
	while (!(SPI1 -> SR & SPI_SR_TXE));
	SPI1 -> DR = (0x8000 | (RegisterNumber << 8));
	
	while (!(SPI1 -> SR & SPI_SR_RXNE));	
	DataInput = SPI1 -> DR;
	
	return (DataInput);
}

/******************************************************************************************************/

void SI3056WriteRegister(uint8 RegisterNumber, uint8 Value){
	
	uint16 DataInput;

	//while (!(SPI1 -> SR & SPI_SR_RXNE));
	//DataInput = SPI1 -> DR;
	
	while (!(SPI1 -> SR & SPI_SR_TXE));
	SPI1 -> DR = 0x0001;
	
	//while (!(SPI1 -> SR & SPI_SR_RXNE));	
	//DataInput = SPI1 -> DR;
	
	while (!(SPI1 -> SR & SPI_SR_TXE));
	SPI1 -> DR = (0x7FFF & ((RegisterNumber << 8) | Value));
	
}

/******************************************************************************************************/

void SendVoiceToPhone(void){
	uint16 Temp;
		
	if(SPI1 -> SR & SPI_SR_RXNE){
		Temp = SPI1 -> DR;
		SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[0]; 
		SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[1];
		
		if (UdpCounter == 128){
			BufferPtr = udp_get_buf (128);
			memcpy(BufferPtr,SItoSPMediaBuffer,128);
			UdpCounter = 0;
			udp_send (UdpMediaSoc, ReceivePacket.IP, UDP_MEDIA_PORT, BufferPtr, 128);
		}
	}
}

/******************************************************************************************************/

void RecieveVoiceFromPhone(void){
	uint16 i;
	uint16 Temp;
	
	if (UdpMediaRecieved == True){
		
		UdpMediaRecieved = False;
		for (i = 0 ; i < 64 ; i++){
			
			if(SPI1 -> SR & SPI_SR_RXNE){
				Temp = SPI1 -> DR;
				SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[0]; 
				SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[1];
				
				if (UdpCounter == 128){
					BufferPtr = udp_get_buf (128);
					memcpy(BufferPtr,SItoSPMediaBuffer,128);
					UdpCounter = 0;
					udp_send (UdpMediaSoc, ReceivePacket.IP, UDP_MEDIA_PORT, BufferPtr, 128);
				}
			}
			
			while (!(SPI1 -> SR & SPI_SR_TXE));
			SPI1 -> DR = ((uint16*)SPtoSIMediaBuffer)[i] & 0xFFFE;
		}
	}
}

/******************************************************************************************************/
void SendDtmfTone(uint16 *DtmfPtr){
	uint16 i;
	for (i = 0 ; i < DTMF_NUM_LEN ; i++){
		while (!(SPI1 -> SR & SPI_SR_TXE));
		SPI1 -> DR = DtmfPtr[i] & 0xFFFE;
		/*if (i % 128 == 0){
			BufferPtr = udp_get_buf (128);
			memcpy(BufferPtr,UdpBuffer,128);
			udp_send (UdpMediaSoc, RemoteIp, UDP_MEDIA_PORT, BufferPtr, 128);
			main_TcpNet();
		}*/
	}
}
/******************************************************************************************************/

/*void SendHelloWorld(void){
	uint8 i;
	uint8 Hello[] = {"\nHello World"}; 

	BufferPtr = udp_get_buf (sizeof(Hello));	
	
	for (i = 0 ; i < sizeof(Hello); i++)
		BufferPtr[i] = Hello[i];
	
	udp_send (udp_media_soc, RemoteIp, UDPPORTNUMBER, BufferPtr, sizeof(Hello));
}*/

/******************************************************************************************************/

void MediaStream(uint8 MediaState){

	if (MediaState == MEDIA_START){
		DmaEnable(DMA2_Stream0, True);															//Enable Si3056 to Smartphone Media Stream	  	(Peripheral to Memory)
	}																				
	else{
		DmaEnable(DMA2_Stream0, False);        													//Disable Si3056 to Smartphone Media Stream     (Peripheral to Memory)																		
		DmaEnable(DMA2_Stream3, False);															//Disable Smartphone to Si3056 Media Stream		(Memory to Peripheral)
	}
}

/******************************************************************************************************/

void OffHook(void){
	SetResetIO(GPIOE, SI_OFHK, enmReset);		//Go to OFF-HOOK
	DeviceState = enmOffHook;
	MediaStream(MEDIA_START);
}

/******************************************************************************************************/

void OnHook(void){
	SetResetIO(GPIOE, SI_OFHK, enmSet);			//Go to ON-HOOK
	DeviceState = enmOnHook;
	MediaStream(MEDIA_STOP);
}

/******************************************************************************************************/

void DiscoveryRoutine(void){
	uint16 CheckSum = 0;
	uint8 *Ptr;
	uint8 i;	

	ReplyPacket = (ReplyPacket_t*)udp_get_buf(DISCOVERY_REPLY_LEN + PACKET_OVERHEAD);
	
	(*ReplyPacket).ID = D2P_ID;
	(*ReplyPacket).Command = ACK;
	(*ReplyPacket).Len = DISCOVERY_REPLY_LEN;
	(*ReplyPacket).Data[0] = (*localm).IpAdr[0];
	(*ReplyPacket).Data[1] = (*localm).IpAdr[1];
	(*ReplyPacket).Data[2] = (*localm).IpAdr[2];
	(*ReplyPacket).Data[3] = (*localm).IpAdr[3];
	
	for (i = 0 ; i < ((*ReplyPacket).Len + PACKET_OVERHEAD - 2) ; i++)
		CheckSum += ((uint8*)ReplyPacket)[i];
	
	(*ReplyPacket).Data[4] = CheckSum;
	(*ReplyPacket).Data[5] = CheckSum >> 8;

	PrintDebug("\nDiscovery Reply Packet Sent ,D2P IP:%d.%d.%d.%d",(*localm).IpAdr[0],(*localm).IpAdr[1],(*localm).IpAdr[2],(*localm).IpAdr[3]);

	udp_send(UdpCtrlSoc, ReceivePacket.IP, UDP_CTRL_PORT, (uint8*)ReplyPacket, PACKET_OVERHEAD + DISCOVERY_REPLY_LEN);
}

/******************************************************************************************************/
void CreationAccountRoutine(void){
	uint16 CheckSum = 0;
	uint8 MacCounter,NameCounter,CurrentSP;
	uint8 i,j;
	uint8 StatusOfCreationAccount = enmCreateAccount;
	
	if (ReceivePacket.ID != 0xFF)		StatusOfCreationAccount = enmGeneralError;
	if (ReceivePacket.Len < MACLEN + 1)		StatusOfCreationAccount = enmGeneralError;
	
	CurrentSP = NumberOfSPs;
	
	for (i = 0 ; i < NumberOfSPs ; i++){
		MacCounter = 0;
		NameCounter = 0;
		
		for (j = 0 ; j < MACLEN ; j++){
			if(ReceivePacket.Data[j] == SpRecord[i].MAC[j]) 	MacCounter++;
		}
		
		for (j = 0 ; j < (ReceivePacket.Len - MACLEN) ; j++){
			if(ReceivePacket.Data[j + MACLEN] == SpRecord[i].Name[j]) 	NameCounter++;
		}
		
		if (NameCounter == SpRecord[i].NameLen){		
			StatusOfCreationAccount = enmDuplicatedName;
			break;
		}
		else if (MacCounter == MACLEN){
			StatusOfCreationAccount = enmRenameAccount;
			CurrentSP = i;
			break;
		}
	}
	
	switch(StatusOfCreationAccount)
	{
		case enmCreateAccount:
		case enmRenameAccount:
			SpRecord[CurrentSP].ID = CurrentSP;
			SpRecord[CurrentSP].NameLen = ReceivePacket.Len - MACLEN;
			memcpy(SpRecord[CurrentSP].MAC,ReceivePacket.Data,MACLEN);
			memcpy(SpRecord[CurrentSP].Name,ReceivePacket.Data + MACLEN,SpRecord[CurrentSP].NameLen);
			memcpy(SpRecord[CurrentSP].IP,ReceivePacket.IP,IPLEN);
			SpRecord[CurrentSP].Status = enmOnLine;
			SpRecord[CurrentSP].Port = ReceivePacket.Port;
			FlashCRLock(False);
			FlashSectoreErase(SPS_RECORD_SECTOR);
			FlashWrite((uint8*)SpRecord, SPS_RECORD_ADDRESS, ((sizeof(SpRecord_ft))*20)/4);
			FlashCRLock(True);
			ReplyPacket = (ReplyPacket_t*)udp_get_buf(ACCOUNT_CREATION_REPLY_LEN + PACKET_OVERHEAD);
			
			(*ReplyPacket).ID = D2P_ID;
			(*ReplyPacket).Command = ACK;
			(*ReplyPacket).Len = ACCOUNT_CREATION_REPLY_LEN;
			(*ReplyPacket).Data[0] = CurrentSP;

		
			for (i = 0 ; i < ((*ReplyPacket).Len + PACKET_OVERHEAD - 2) ; i++)
				CheckSum += ((uint8*)ReplyPacket)[i];
			if (StatusOfCreationAccount == enmRenameAccount){
				PrintDebug("\nSP Account Created with");
			}
			else{
				PrintDebug("\nSP Renamed to");
			}
			PrintDebug("\n\t\tID: %d \n\t\tName: %s",CurrentSP,SpRecord[CurrentSP].Name)
			PrintDebug("\n\t\tMAC: %02X:%02X:%02X:%02X:%02X:%02X",SpRecord[CurrentSP].MAC[0],SpRecord[CurrentSP].MAC[1],SpRecord[CurrentSP].MAC[2],SpRecord[CurrentSP].MAC[3],SpRecord[CurrentSP].MAC[4],SpRecord[CurrentSP].MAC[5]);

			udp_send(UdpCtrlSoc, ReceivePacket.IP, UDP_CTRL_PORT, (uint8*)ReplyPacket, ACCOUNT_CREATION_REPLY_LEN + DISCOVERY_REPLY_LEN);
			NumberOfSPs++;
			break;	
			
		case enmDuplicatedName:
		case enmGeneralError:	
			(*ReplyPacket).ID = D2P_ID;
			(*ReplyPacket).Command = NACK;
			(*ReplyPacket).Len = NACK_REPLY_LEN;
			(*ReplyPacket).Data[0] = StatusOfCreationAccount;
		
			for (i = 0 ; i < ((*ReplyPacket).Len + PACKET_OVERHEAD - 2) ; i++)
				CheckSum += ((uint8*)ReplyPacket)[i];

			if (StatusOfCreationAccount == enmDuplicatedName) {
				PrintDebug("\nSP Name is Duplicated");
			}
			else{
				 PrintDebug("\nCreation Account Error");
			}
			
			udp_send(UdpCtrlSoc, ReceivePacket.IP, UDP_CTRL_PORT, (uint8*)ReplyPacket, NACK_REPLY_LEN + NACK_REPLY_LEN);
			break;
	}
	
}

/******************************************************************************************************/
void CallRoutine(void){
	uint16 CheckSum = 0;
	uint8 i;	

	ReplyPacket = (ReplyPacket_t*)udp_get_buf(ACK_REPLY_LEN + PACKET_OVERHEAD);
	
	(*ReplyPacket).ID = D2P_ID;
	(*ReplyPacket).Command = ACK;
	(*ReplyPacket).Len = ACK_REPLY_LEN;
	
	for (i = 0 ; i < ((*ReplyPacket).Len + PACKET_OVERHEAD - 2) ; i++)
		CheckSum += ((uint8*)ReplyPacket)[i];
	
	(*ReplyPacket).Data[0] = CheckSum;
	(*ReplyPacket).Data[1] = CheckSum >> 8;
	
	udp_send(UdpCtrlSoc, ReceivePacket.IP, UDP_CTRL_PORT, (uint8*)ReplyPacket, PACKET_OVERHEAD);
	
	PrintDebug("\nCalling: ");
	for (i = 0 ; i < ReceivePacket.Len ; i++)
		PrintDebug("%d",ReceivePacket.Data[i]);
	PrintDebug("...");
	
	OffHook();
	
	delay(200);
	
	for (i = 0 ; i < ReceivePacket.Len ; i++){
		SendDtmfTone((uint16*)DtmfCode[ReceivePacket.Data[i]]);
		delay(22);
	}
}

/******************************************************************************************************/

void AnswerRoutine(void){
	uint16 CheckSum = 0;
	uint8 i;	

	ReplyPacket = (ReplyPacket_t*)udp_get_buf(ACK_REPLY_LEN + PACKET_OVERHEAD);
	
	(*ReplyPacket).ID = D2P_ID;
	(*ReplyPacket).Command = ACK;
	(*ReplyPacket).Len = ACK_REPLY_LEN;
	
	for (i = 0 ; i < ((*ReplyPacket).Len + PACKET_OVERHEAD - 2) ; i++)
		CheckSum += ((uint8*)ReplyPacket)[i];
	
	(*ReplyPacket).Data[0] = CheckSum;
	(*ReplyPacket).Data[1] = CheckSum >> 8;
	
	PrintDebug("\nAnswer...");

	OffHook();
	delay(200);

	udp_send(UdpCtrlSoc, ReceivePacket.IP, UDP_CTRL_PORT, (uint8*)ReplyPacket, PACKET_OVERHEAD);
}

/******************************************************************************************************/
void HangUpRoutine(void){
	uint16 CheckSum = 0;
	uint8 i;	

	ReplyPacket = (ReplyPacket_t*)udp_get_buf(ACK_REPLY_LEN + PACKET_OVERHEAD);
	
	(*ReplyPacket).ID = D2P_ID;
	(*ReplyPacket).Command = ACK;
	(*ReplyPacket).Len = ACK_REPLY_LEN;
	
	for (i = 0 ; i < ((*ReplyPacket).Len + PACKET_OVERHEAD - 2) ; i++)
		CheckSum += ((uint8*)ReplyPacket)[i];
	
	(*ReplyPacket).Data[0] = CheckSum;
	(*ReplyPacket).Data[1] = CheckSum >> 8;
	
	PrintDebug("\nHang Up");

	OnHook();
	delay(200);

	udp_send(UdpCtrlSoc, ReceivePacket.IP, UDP_CTRL_PORT, (uint8*)ReplyPacket, PACKET_OVERHEAD);
}

/******************************************************************************************************/
/*																									  */
/*										Interrrupt Functions										  */
/*																									  */
/******************************************************************************************************/

//Interrupt DMA2 Stream0
void DMA2_Stream0_IRQHandler(void){
	
	if ((DMA2 -> LISR & DMA_LISR_TCIF0) == DMA_LISR_TCIF0)
	{
		BufferPtr = udp_get_buf (UDP_PACKET_SIZE);
		memcpy(BufferPtr,SItoSPMediaBuffer,UDP_PACKET_SIZE);
		udp_send (UdpMediaSoc, ReceivePacket.IP, UDP_MEDIA_PORT, BufferPtr, UDP_PACKET_SIZE);
		DmaEnable(DMA2_Stream0, True);															//Enable Si3056 to Smartphone Media Stream	  	(Peripheral to Memory)
	}
}


