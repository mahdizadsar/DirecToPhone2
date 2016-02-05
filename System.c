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
uint8 			SampleCounter = 0;

//uint16			TxValue[24000];
//uint16			RxValue[24000];
uint16			RxCounter = 0;
uint16			TxCounter = 0;

uint8 			Record = 0;
uint32 IntCtr = 0;
uint32 TxCtr = 0;

uint32 Jcounter = 0;

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

extern int16				Tone1KHz[];
extern uint8 BufFlag;

/******************************************************************************************************/
//Functions
void ReadSPsRecord(void){
	uint8 i;
	
	NumberOfSPs = 0;
	
	for (i = 0 ; i < 20 ; i++){
		if (SpRecordFlash[i].ID != 0xFF)	NumberOfSPs++;
		memcpy((uint8*)&SpRecord[i], (uint8*)&SpRecordFlash[i], sizeof(SpRecord_ft));
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
	
	DataInput = SPI1 -> DR;
	
 	while (!(SPI1 -> SR & SPI_SR_RXNE) || (SPI1 -> SR & SPI_SR_BSY)); 		
 	DataInput = SPI1 -> DR;
	
	while (!(SPI1 -> SR & SPI_SR_TXE) || (SPI1 -> SR & SPI_SR_BSY));
	SPI1 -> DR = 0x0001;
	
	while (!(SPI1 -> SR & SPI_SR_RXNE) || (SPI1 -> SR & SPI_SR_BSY));
	DataInput = SPI1 -> DR;
	
	while (!(SPI1 -> SR & SPI_SR_TXE) || (SPI1 -> SR & SPI_SR_BSY));
	SPI1 -> DR = (0x8000 | (RegisterNumber << 8));
	
	while (!(SPI1 -> SR & SPI_SR_RXNE) || (SPI1 -> SR & SPI_SR_BSY));	
	DataInput = SPI1 -> DR;
	
	return (DataInput);
}

/******************************************************************************************************/

void SI3056WriteRegister(uint8 RegisterNumber, uint8 Value){
	
	uint16 DataInput;

	//while (!(SPI1 -> SR & SPI_SR_RXNE));
	//DataInput = SPI1 -> DR;
	
	while (!(SPI1 -> SR & SPI_SR_TXE) || (SPI1 -> SR & SPI_SR_BSY));
	SPI1 -> DR = 0x0001;
	
	//while (!(SPI1 -> SR & SPI_SR_RXNE));	
	//DataInput = SPI1 -> DR;
	
	while (!(SPI1 -> SR & SPI_SR_TXE) || (SPI1 -> SR & SPI_SR_BSY));
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
	static uint16 i = 0;
	uint16 Temp;
	
	//if (UdpMediaRecieved == True){
		
	//	UdpMediaRecieved = False;
	
	
	
		if (UdpMediaRecieved == True){
		
		//UdpMediaRecieved = False;
		//for (i = 0 ; i < 64 ; i++){
			
			if(SPI1 -> SR & SPI_SR_RXNE){
				Temp = SPI1 -> DR;
				
				//if (Record == 1 && RxCounter < 24000)	RxValue[RxCounter++] = Temp;
				
				SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[0]; 
				SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[1];
				
				if (UdpCounter == 128){
					BufferPtr = udp_get_buf (128);
					memcpy(BufferPtr,SItoSPMediaBuffer,128);
					UdpCounter = 0;
					udp_send (UdpMediaSoc, ReceivePacket.IP, UDP_MEDIA_PORT, BufferPtr, 128);
				}
			}
			
			
			//while (!(SPI1 -> SR & SPI_SR_TXE));
			if(((SPI1 -> SR & SPI_SR_TXE))){
				SPI1 -> DR = ((uint16*)SPtoSIMediaBuffer)[i] & 0xFFFE;
				if (i++ == 64) {
					UdpMediaRecieved = False;
					i = 0;
				}
				
				
				//if (Record == 1 && TxCounter < 24000)	TxValue[TxCounter++] = ((uint16*)SPtoSIMediaBuffer)[i];
			}
			
		//}
	}
	
	
	
//		for (i = 0 ; i < 64 ; i++){
/*			
			if(SPI1 -> SR & SPI_SR_RXNE){
				Temp = SPI1 -> DR;
				
				if (Record == 1 && RxCounter < 24000)	RxValue[RxCounter++] = Temp;
				
					
				SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[0]; 
				SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[1];
				
				if (UdpCounter == 128){
					BufferPtr = udp_get_buf (128);
					memcpy(BufferPtr,SItoSPMediaBuffer,128);
					UdpCounter = 0;
					udp_send (UdpMediaSoc, ReceivePacket.IP, UDP_MEDIA_PORT, BufferPtr, 128);
				}
			}
			
			//while (!(SPI1 -> SR & SPI_SR_TXE));
			if (UdpMediaRecieved == True){
				if ((SPI1 -> SR & SPI_SR_TXE))
				{	
					
					if (Record == 1 && TxCounter < 24000)	TxValue[TxCounter++] = Tone1KHz[SampleCounter];
					
					SPI1 -> DR = ((uint16*)SPtoSIMediaBuffer)[SampleCounter] & 0xFFFE; //Tone1KHz[SampleCounter]
					//if (SampleCounter++ == 16)
					
					SampleCounter++;
					
					if (SampleCounter == 64) {
						UdpMediaRecieved = False;
						SampleCounter = 0;
					}
				}
			}
*/
	//}
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
	uint32 x;
	SetResetIO(GPIOE, SI_OFHK, enmReset);		//Go to OFF-HOOK
	DeviceState = enmOffHook;
	MediaStream(MEDIA_START);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	
	
	//NVIC_EnableIRQ(SPI1_IRQn);
	//SPI1 -> CR2 |= SPI_CR2_RXNEIE; 
	//x = SPI1 -> SR;
	//TxCtr = 0;
	//UdpCounter = 0;
}

/******************************************************************************************************/

void OnHook(void){
	uint32 x;
	SetResetIO(GPIOE, SI_OFHK, enmSet);			//Go to ON-HOOK
	DeviceState = enmOnHook;
	//x = SPI1 -> SR;
	//NVIC_DisableIRQ(SPI1_IRQn);
	//SPI1 -> CR2 = 0;
	//UdpMediaRecieved = False;
	NVIC_DisableIRQ(DMA2_Stream0_IRQn);
	NVIC_DisableIRQ(DMA2_Stream3_IRQn);
	
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
		
		if (MacCounter == MACLEN){
			StatusOfCreationAccount = enmRenameAccount;
			CurrentSP = i;
			break;
		}else if (SpRecord[i].NameLen == NameCounter && SpRecord[i].NameLen == (ReceivePacket.Len - MACLEN)){		
			StatusOfCreationAccount = enmDuplicatedName;
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
			
			for (i = 0 ; i < 20 ; i++)
				FlashWrite((uint32*)&SpRecord[i], (uint32*)(SPS_RECORD_ADDRESS + (i * sizeof(SpRecord_ft))), (sizeof(SpRecord_ft))/4);
			
			FlashCRLock(True);
			ReplyPacket = (ReplyPacket_t*)udp_get_buf(ACCOUNT_CREATION_REPLY_LEN + PACKET_OVERHEAD);
			
			(*ReplyPacket).ID = D2P_ID;
			(*ReplyPacket).Command = ACK;
			(*ReplyPacket).Len = ACCOUNT_CREATION_REPLY_LEN;
			(*ReplyPacket).Data[0] = CurrentSP;

		
			for (i = 0 ; i < ((*ReplyPacket).Len + PACKET_OVERHEAD - 2) ; i++)
				CheckSum += ((uint8*)ReplyPacket)[i];
			if (StatusOfCreationAccount == enmCreateAccount){
				PrintDebug("\nSP Account Created with");
				NumberOfSPs++;
			}
			else{
				PrintDebug("\nSP Renamed to");
			}
			PrintDebug("\n\t\tID: %d \n\t\tName: ",CurrentSP);
			for (i = 0 ; i < SpRecord[CurrentSP].NameLen ; i++)
				PrintDebug("%c",SpRecord[CurrentSP].Name[i]);

			PrintDebug("\n\t\tMAC: %02X:%02X:%02X:%02X:%02X:%02X",SpRecord[CurrentSP].MAC[0],SpRecord[CurrentSP].MAC[1],SpRecord[CurrentSP].MAC[2],SpRecord[CurrentSP].MAC[3],SpRecord[CurrentSP].MAC[4],SpRecord[CurrentSP].MAC[5]);

			udp_send(UdpCtrlSoc, ReceivePacket.IP, UDP_CTRL_PORT, (uint8*)ReplyPacket, ACCOUNT_CREATION_REPLY_LEN + DISCOVERY_REPLY_LEN);
			
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
	PrintDebug("\n-----------------");
	PrintDebug("\nNumber of SPs: %d",NumberOfSPs);
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
	SampleCounter = 0;
	delay(200);
	
// 	for (i = 0 ; i < ReceivePacket.Len ; i++){
// 		SendDtmfTone((uint16*)DtmfCode[ReceivePacket.Data[i]]);
// 		delay(22);
// 	}
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

//Interrupt SPI1 TX and RX
void SPI1_IRQHandler(void){
	uint16 Temp;
	uint32 temp2;
		
	if((SPI1 -> SR & SPI_SR_TXE) && UdpMediaRecieved == True){
			
		if (BufFlag == 0)
			BufFlag = 1;
		else
			BufFlag = 0;
	
		SPI1 -> DR = ((uint16*)SPtoSIMediaBuffer)[TxCtr++] & 0xFFFE;
		
		if (TxCtr == 63) {
			TxCtr = 0;
			Jcounter++;
		}
		
		//if (Record == 1 && TxCounter < 24000)	TxValue[TxCounter++] = ((uint16*)SPtoSIMediaBuffer)[TxCtr];
	}
	
	/*if(SPI1 -> SR & SPI_SR_RXNE){
		Temp = SPI1 -> DR;

		if (Record == 1 && RxCounter < 24000)	RxValue[RxCounter++] = Temp;
		
		SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[0]; 
		SItoSPMediaBuffer[UdpCounter++] = ((uint8*)&Temp)[1];
		
		if (UdpCounter == 128){
			BufferPtr = udp_get_buf (128);
			memcpy(BufferPtr,SItoSPMediaBuffer,128);
			UdpCounter = 0;
			udp_send (UdpMediaSoc, ReceivePacket.IP, UDP_MEDIA_PORT, BufferPtr, 128);
		}
	}*/
	
	
}



