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
uint8 *BufferPtr;
uint8 UdpBuffer[512];

/******************************************************************************************************/
//Global Constant
uint8 RemoteIp[4] = {192,168,1,3};

/******************************************************************************************************/
//External Variables
extern uint8 udp_media_soc;
extern uint8 UdpMediaRecieved;
extern uint8 MediaBuffer[256];

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
	
	while (!(SPI1 -> SR & SPI_SR_RXNE));
	DataInput = SPI1 -> DR;
	
	while (!(SPI1 -> SR & SPI_SR_TXE));
	SPI1 -> DR = 0x0001;
	
	while (!(SPI1 -> SR & SPI_SR_RXNE));	
	DataInput = SPI1 -> DR;

	while (!(SPI1 -> SR & SPI_SR_TXE));
	SPI1 -> DR = (0x7FFF & ((RegisterNumber << 8) | Value));

}

/******************************************************************************************************/

void SendVoiceToPhone(void){
	static uint16 i = 0;
	uint16 Temp;
		
	if(SPI1 -> SR & SPI_SR_RXNE){
		Temp = SPI1 -> DR;
		UdpBuffer[i++] = ((uint8*)&Temp)[0]; 
		UdpBuffer[i++] = ((uint8*)&Temp)[1];
		
		if (i == 128){
			BufferPtr = udp_get_buf (128);
			memcpy(BufferPtr,UdpBuffer,128);
			i = 0;
			udp_send (udp_media_soc, RemoteIp, UDPMEDIAPORT, BufferPtr, 128);
		}
	}
}

/******************************************************************************************************/

void RecieveVoiceFromPhone(void){
	uint16 i;
	if (UdpMediaRecieved == True){
		UdpMediaRecieved = False;
		for (i = 0 ; i < 64 ; i++){
			while (!(SPI1 -> SR & SPI_SR_TXE));
			SPI1 -> DR = ((uint16*)MediaBuffer)[i];
		}
	}
}

/******************************************************************************************************/
void SendDtmfTone(uint16 *DtmfPtr){
	uint16 i;
	for (i = 0 ; i < 800 ; i++){
		while (!(SPI1 -> SR & SPI_SR_TXE));
		SPI1 -> DR = DtmfPtr[i] & 0xFFFE;
		if (i % 128 == 0){
			BufferPtr = udp_get_buf (128);
			memcpy(BufferPtr,UdpBuffer,128);
			udp_send (udp_media_soc, RemoteIp, UDPMEDIAPORT, BufferPtr, 128);
			main_TcpNet();
		}
	}
}
/******************************************************************************************************/

void SendHelloWorld(void){
	uint8 i;
	uint8 Hello[] = {"\nHello World"}; 

	BufferPtr = udp_get_buf (sizeof(Hello));	
	
	for (i = 0 ; i < sizeof(Hello); i++)
		BufferPtr[i] = Hello[i];
	
	udp_send (udp_media_soc, RemoteIp, UDPPORTNUMBER, BufferPtr, sizeof(Hello));
}

/******************************************************************************************************/



    









