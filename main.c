/*
#####################################################################################
#								In the name of ALLAH 								#
# Project : Direct to Phone															#
# Date : 2015/06/04																	#
# file : Main.c 																	#
#																					#
#####################################################################################
*/

#include "stm32f4xx.h"
#include "types.h"
#include "SystemInitial.h"
//#include "Ethernet\ETH_STM32F4xx.h"
#include "stdio.h"
#include "RTL.h"
#include "GeneralDefines.h" 
#include "Net_Config.h"

extern void SI3056WriteRegister(uint8 ,uint8 );
/******************************************************************************************************/
//Global Defines
#define NUM_RX_BUF          4           /* 0x1800 for Rx (4*1536=6K)         */
#define NUM_TX_BUF          2           /* 0x0600 for Tx (2*1536=3K)         */
#define ETH_BUF_SIZE        1536        /* ETH Receive/Transmit buffer size  */

/******************************************************************************************************/
//Global Variables
uint8 				UdpCtrlSoc;				//UDP Signaling Handler
uint8 				UdpMediaSoc;			//UDP Media Handler
//--------------------------------------
SpRecord_t 						SpRecord[20];
__align(4) SpRecord_ft			SpRecordFlash[20]		__attribute__((at(SPS_RECORD_ADDRESS)));

ReceivePacket_t 	ReceivePacket;


uint32 				i,j,k,l;


uint8 				udp_media_soc;
uint16 				DataRead[3],Data3;
uint16 				TCPRxTcpDataCount;
uint8 				*TCPRxDataPtr;
boolean 			DataReceivedFlag;
uint8 				SPtoSIMediaBuffer[128];
uint8 				SItoSPMediaBuffer[128];
enmDeviceState_n 	DeviceState;
uint8 				UdpRecieved;
uint8 				UdpMediaRecieved;
uint8 				*BufferPtr2;
uint8 				NumberOfSPs;

/******************************************************************************************************/
//External Variables
extern uint8 				DtmfCode[10][1600];
extern LOCALM 				localm[];

/******************************************************************************************************/

U16 UdpCtrlCallback (U8 socket, U8 *remip, U16 remport, U8 *buf, U16 len) {
  /* This function is called when UDP data is received */
	
  /* Process received data from 'buf' */
	memcpy(ReceivePacket.IP, remip, 4);
	memcpy((uint8*)&ReceivePacket, buf, (len - 2));
	memcpy((uint8*)&ReceivePacket.CheckSum, (buf + len - 2), 2);
	ReceivePacket.Port = remport;
	UdpRecieved = True;
	
	return (0);
}

/******************************************************************************************************/

U16 UdpMediaCallback (U8 socket, U8 *remip, U16 remport, U8 *buf, U16 len) {
  /* This function is called when UDP data is received */
	
  /* Process received data from 'buf' */
	memcpy(SPtoSIMediaBuffer,buf,len);
	//UdpMediaRecieved = True;
	
	if (DeviceState == enmOffHook)			
		DmaEnable(DMA2_Stream3, True);														//Enable Smartphone to Si3056 Media Stream Again (Memory to Peripheral)
	
	return (0);
}

/******************************************************************************************************/

void delay (uint32 time){
	uint32 i;	
	for (i = 0 ; i < time * 0x20000 ; i++);
}

/******************************************************************************************************/

void LED1(void){
	while(1){
		for (k = 15 ; k > 1 ; k--) {
			for (l = 12 ; l < 16 ; l++){
				GPIOD -> ODR = (uint32)(1 << l);
				delay(8 * k);
			}
		}
		
		for (j = 0 ; j < 20 ; j++){
			for (l = 12 ; l < 16 ; l++){
				GPIOD -> ODR = (uint32)(1 << l);
				delay(8 * k);
			}
		}
		
		for (k = 1 ; k < 15 ; k++){
			for (l = 12 ; l < 16 ; l++){
				GPIOD -> ODR = (uint32)(1 << l);
				delay(8 * k);
			}
		}
	}
}

/******************************************************************************************************/

void LED2 (void){
	while(1){
		GPIOD -> ODR = (1 << 12)|(1 << 5);
		delay(20);
		GPIOD -> ODR = (1 << 5);
		delay(1000);
		
		GPIOD -> ODR = (1 << 13)|(1 << 5);
		delay(20);
		GPIOD -> ODR = (1 << 5);
		delay(1000);
		
		GPIOD -> ODR = (1 << 14)|(1 << 5);
		delay(20);
		GPIOD -> ODR = (1 << 5);
		delay(1000);
		
		GPIOD -> ODR = (1 << 15)|(1 << 5);
		delay(20);
		GPIOD -> ODR = (1 << 5);
		delay(50);
		
		GPIOD -> ODR = (0 << 5);
		delay(20);
		GPIOD -> ODR = (1 << 5);
		delay(1000);
	}
}

/******************************************************************************************************/

void SER_PutChar(uint8 Data){
	
		while(!(USART3 -> SR & USART_SR_TXE));
		USART3 -> DR = Data; 
	}


/******************************************************************************************************/

void UartTransmmit(uint8 *Data, uint32 Size, uint8 Channel){
	for (i = 0 ; i < Size ; i++){
		while(!(USART1 -> SR & USART_SR_TXE));
		USART1 -> DR = Data[i]; 
	}
}

/******************************************************************************************************/

void UartTransmmit3(uint8 *Data, uint32 Size, uint8 Channel){
	for (i = 0 ; i < Size ; i++){
		while(!(USART3 -> SR & USART_SR_TXE));
		USART3 -> DR = Data[i]; 
	}
}

/******************************************************************************************************/

void send_data (uint8 Char) {
/*	uint8 udp_msg;
	uint8 remip[4] = {192,168,1,6};
	uint8 *sendbuf;
	uint16 len=5;

	udp_msg = Char;

	sendbuf = udp_get_buf (1);
	memcpy (sendbuf, &udp_msg,1);
	// Send 'Hello World!' to remote peer 
	udp_send (udp_soc, remip, 1000, sendbuf, 1);
	main_TcpNet();*/
}

/******************************************************************************************************/

void RegistrationFunction(void){
/*	memcpy(UserRecord.MAC, Packet.Data, MACLEN);
	memcpy(UserRecord.Name, Packet.Data + MACLEN, NAMELEN);
	memcpy(UserRecord.IP, Packet.IP, IPLEN);
	
	Packet.Command = enmRegisterAccept;
	Packet.CheckSum = 0;
	
	for (i = 0; i < 29 ; i++)
		Packet.CheckSum += ((uint8*)&Packet.Command)[i]; 
	PrintDebug("\nRegistration Packet Sent");
	
	BufferPtr2 = udp_get_buf (31);
	memcpy(BufferPtr2,&Packet.Command,31);
	
	udp_send (udp_soc, Packet.IP , UDPSIGNALINGPORT, BufferPtr2, 31);*/
}

/******************************************************************************************************/
void CallFunction(void){
/*	PrintDebug("\nCall Number:");
	for (i = 0 ; i < Packet.Len ; i++)
		PrintDebug("%c",Packet.Data[i]);
	
	OffHook();
	for (i = 0 ; i < Packet.Len ; i++){
		//SendDtmfTone((uint16*)DtmfCode[Packet.Data[i]]);
		delay(50);
	}*/
}

/******************************************************************************************************/

int main(){

	uint32 Counter = 0 , IPCounter = 0; 
	uint16 maxlen;
	uint16 i;
	uint8 *sendbuf;
	uint32 timer = 0;
	uint8 CID[20];
	uint16 CIDCounter = 0;
	uint8 OneTimeFlag = False;
// 	uint32 *Memory = (uint32*)0x8000000;
// 	UartTransmmit3(Data, sizeof(Data), 0);
// 	DmaConfig(DMA2_Stream7, (uint32)&(USART1 -> DR), (uint32)Data1 , 0 ,sizeof(Data1) - 1);
// 	DmaEnable(DMA2_Stream7, True);
	
	USART3 -> BRR = 0x00000145;
	
	ReadSPsRecord();
	
	init_TcpNet();

	UdpCtrlSoc = udp_get_socket(0, UDP_OPT_SEND_CS | UDP_OPT_CHK_CS, UdpCtrlCallback);
	if (UdpCtrlSoc != 0) {
		/* Open UDP port 5060 for Signaling communication */
		udp_open (UdpCtrlSoc, UDP_CTRL_PORT);
	}
 
	UdpMediaSoc = udp_get_socket(0, UDP_OPT_SEND_CS | UDP_OPT_CHK_CS, UdpMediaCallback);
	if (UdpMediaSoc != 0) {
		/* Open UDP port 8000 for Media communication */
		udp_open (UdpMediaSoc, UDP_MEDIA_PORT);
	}
	
//	delay(5000);
	PrintDebug("\n*** In The Name of ALLAH ***");
	SystemConfiguration();
//	PrintDebug("\nSystem Configuration Done");
//	PrintDebug("\nSystem Startup");
//	FlashOptionByteCRLock(False);
	SI3056WriteRegister(6, 0x00);
	delay(100);
	SI3056WriteRegister(8, 0x02);
	SI3056WriteRegister(9, 0x13);
	delay(200);
	SI3056WriteRegister(1, 0x28);
	SI3056WriteRegister(20, 0xFF);
	SI3056WriteRegister(21, 0xFF);
	SI3056WriteRegister(5, 0x02);
	SI3056WriteRegister(7, 0x09);				//Set Codec to 16Kbps
	
	SI3056WriteRegister(45, 0xFF);
	SI3056WriteRegister(46, 0xFF);
	SI3056WriteRegister(47, 0xFF);
	SI3056WriteRegister(48, 0xFF);
	SI3056WriteRegister(49, 0xFF);
	SI3056WriteRegister(50, 0xFF);
	SI3056WriteRegister(51, 0xFF);
	SI3056WriteRegister(52, 0xFF);
	
	//PrintDebug("\nReading Si3056 Registers");
	SetResetIO(GPIOE, SI_OFHK, enmReset);		//Go to OFF-HOOK
	delay(500);
	for (i = 1 ; i < 60 ; i++){	
		PrintDebug("\nSi3056 Register%02d: %02X", i ,SI3056ReadRegister(i));
		delay(2);
	}
	SetResetIO(GPIOE, SI_OFHK, enmSet);			//Go to ON-HOOK

	
	DmaConfig(DMA2_Stream0,(uint32)&SPI1 -> DR, (uint32)SItoSPMediaBuffer, 0, UDP_PACKET_SIZE/2);
	DmaConfig(DMA2_Stream3,(uint32)&SPI1 -> DR, (uint32)SPtoSIMediaBuffer, 0, UDP_PACKET_SIZE/2);

	
	//PrintDebug("\nSend DTMF 0 to Si3056");
	/*for (i = 0 ; i < 11 ; i++){
		SendDtmfTone((uint16*)DtmfCode[PhoneNumber[i]]);
		delay(50);
	}
	delay(5000);*/
	//PrintDebug("\nEnd of Send Si3056");

	/*while(1){
		while (!(SPI1 -> SR & SPI_SR_RXNE));
		DataRead[0] = SPI1 -> DR;
		PrintDebug("\nDataRead:%04X",DataRead[0]);
		delay(1);
	}*/

	GPIOE -> MODER &= ~(3 << 22);
	GPIOD -> ODR |= (1 << HT9032_PWRDOWN);

 	while(1){
 		/*if ((Rx_Desc[0].Stat & 0x80000000) == 0){
			for (i = 0 ; i < ((Rx_Desc[0].Stat >> 16) & 0x3FFF) ; i++)
				PrintDebug("%04X" , ((uint8*)(Rx_Desc[0].Addr))[i]);
		}*/
		main_TcpNet();
		//SendHelloWorld();
		
		timer++;
		if (timer == 35000){
			timer = 0;
			timer_tick();
			if(OneTimeFlag == False)	IPCounter++;
			if(OneTimeFlag == False && IPCounter == 5){
				PrintDebug("\n\nD2P IP:%d.%d.%d.%d",(*localm).IpAdr[0],(*localm).IpAdr[1],(*localm).IpAdr[2],(*localm).IpAdr[3]);
				OneTimeFlag = True;
			}
		}
		if (!(GPIOE -> IDR & (1 << 11))){
			GPIOD -> ODR = (1 << 14);
			Counter++;
		}
		else{
			GPIOD -> ODR &= ~(1 << 14);
		}
	

		
// 		if (!(GPIOE -> IDR & (1 << 11)/*GPIOD -> IDR & (1 << HT9032_RING)*/)){
// 			//delay(100);
// 			GPIOD -> ODR &= ~(1 << HT9032_PWRDOWN);
// 			PrintDebug("\nTel Ringing...");
// 			do{
// 				if(USART3 -> SR & USART_SR_RXNE){
// 					CID[CIDCounter] = USART3 -> DR;
// 					CIDCounter++;
// 				}
// 			}while(CIDCounter < 300);
// 			CIDCounter = 0;
// 			PrintDebug("\nCID:");
// 			for (i = 0 ; i < 300 ; i++)
// 				PrintDebug("%d",CID[i]);
// 		}	
		
		
		
		/*if (Counter > 100000){
			Counter = 0;
			SetResetIO(GPIOE, SI_OFHK, enmReset);
			delay(2000);
			SetResetIO(GPIOE, SI_OFHK, enmSet);
		}*/
		
		if (UdpRecieved == True) {
			UdpCommandParser();
			UdpRecieved = False;
		}
		
// 		if (DeviceState == enmOffHook) {
// 			SendVoiceToPhone();
// 			RecieveVoiceFromPhone();
// 		}
 			
		/*if (DataReceivedFlag == True){
			DataReceivedFlag = False;
			switch(TCPRxDataPtr[0]){
				case 0x00:
					if (TCPRxTcpDataCount == 1)
						SetResetIO(GPIOE, SI_OFHK, enmSet);			//Go to ON-HOOK
					DeviceState = enmOnHook;
					
					if (tcp_check_send (tcp_soc)) {
						sendbuf = tcp_get_buf (sizeof(Reply));
						memcpy (sendbuf, Reply, sizeof(Reply));
						tcp_send (tcp_soc, sendbuf, sizeof(Reply));
					}	
					GPIOD -> ODR = 0x00;
				break;
					
				case 0x01:
					if (TCPRxTcpDataCount == 1)
						SetResetIO(GPIOE, SI_OFHK, enmReset);		//Go to OFF-HOOK
					DeviceState = enmOffHook;
					
					if (tcp_check_send (tcp_soc)) {
						sendbuf = tcp_get_buf (sizeof(Reply));
						memcpy (sendbuf, Reply, sizeof(Reply));
						tcp_send (tcp_soc, sendbuf, sizeof(Reply));
					}
					GPIOD -> ODR = (1 << 12);
				break;
				
//-------------------------------------------------------------------------------------------------					
								
				case 0x11:
					PrintDebug("\n*** In The Name of ALLAH ***");
					GPIOD -> ODR = (1 << 13);
				break;
				case 0x12:
					GPIOD -> ODR = (1 << 14);
				break;
				case 0x13:
					GPIOD -> ODR = (1 << 15);
				break;
			}
		}*/
	}
	//LED1();
}





