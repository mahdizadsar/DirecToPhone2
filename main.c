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
 
/******************************************************************************************************/
//Global Defines
#define NUM_RX_BUF          4           /* 0x1800 for Rx (4*1536=6K)         */
#define NUM_TX_BUF          2           /* 0x0600 for Tx (2*1536=3K)         */
#define ETH_BUF_SIZE        1536        /* ETH Receive/Transmit buffer size  */

/******************************************************************************************************/
//Global Variables

uint32 i,j,k,l;
uint8 *TempLoc		__attribute__((at(0x8000000 + 0x4000)));

uint8 tcp_soc;
uint8 udp_soc;
uint16 DataRead;
uint16 TCPRxTcpDataCount;
uint8 *TCPRxDataPtr;
boolean DataReceivedFlag;

/******************************************************************************************************/
uint16 tcp_callback (uint8 soc, uint8 event, uint8 *ptr, uint16 par) {
	/* This function is called on TCP event */

	switch (event) {
	case TCP_EVT_CONREQ:
		/* Remote host is trying to connect to our TCP socket. */
		/* 'ptr' points to Remote IP, 'par' holds the remote port. */

		/* Return 1 to accept connection, or 0 to reject connection */
		return (1);
	case TCP_EVT_ABORT:
		/* Connection was aborted */

		break;
	case TCP_EVT_CONNECT:
		/* Socket is connected to remote peer. */

		break;
	case TCP_EVT_CLOSE:
		/* Connection has been closed */

		break;
	case TCP_EVT_ACK:
		/* Our sent data has been acknowledged by remote peer */

		break;
	case TCP_EVT_DATA:
		/* TCP data frame has been received, 'ptr' points to data */
		/* Data length is 'par' bytes */
		TCPRxTcpDataCount = par;
		TCPRxDataPtr = ptr;
		DataReceivedFlag = True;
		break;
	}
	return (0);
}

/******************************************************************************************************/

U16 udp_callback (U8 socket, U8 *remip, U16 remport, U8 *buf, U16 len) {
  /* This function is called when UDP data is received */

  /* Process received data from 'buf' */
   
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
	
		while(!(USART1 -> SR & USART_SR_TXE));
		USART1 -> DR = Data; 
	}


/******************************************************************************************************/

void UartTransmmit(uint8 *Data, uint32 Size, uint8 Channel){
	for (i = 0 ; i < Size ; i++){
		while(!(USART1 -> SR & USART_SR_TXE));
		USART1 -> DR = Data[i]; 
	}
}

/******************************************************************************************************/

void send_data (uint8 Char) {
	uint8 udp_msg;
	uint8 remip[4] = {192,168,1,6};
	uint8 *sendbuf;
	uint16 len=5;

	udp_msg = Char;

	sendbuf = udp_get_buf (1);
	memcpy (sendbuf, &udp_msg,1);
	/* Send 'Hello World!' to remote peer */
	udp_send (udp_soc, remip, 1000, sendbuf, 1);
	main_TcpNet();
}

/******************************************************************************************************/

int main(){
	uint16 maxlen;
	
	uint8 *sendbuf;
	uint32 timer = 0;
 	uint8 Data[] = {"\n*** In The Name of ALLAH ***"};
// 	uint8 Data1[] = {"\nDMA USART1 TX Initialized....."};
// 	uint32 *Memory = (uint32*)0x8000000;

// 	UartTransmmit(Data, sizeof(Data), 0);
// 	DmaConfig(DMA2_Stream7, (uint32)&(USART1 -> DR), (uint32)Data1 , 0 ,sizeof(Data1) - 1);
// 	DmaEnable(DMA2_Stream7, True);
	
	
	
	//printf("\n*** In The Name of ALLAH ***");
	
	init_TcpNet();
	udp_soc = udp_get_socket(0, UDP_OPT_SEND_CS | UDP_OPT_CHK_CS, udp_callback);
	if (udp_soc != 0) {
		/* Open UDP port 1000 for communication */
		udp_open (udp_soc, 1000);
	}

  
	tcp_soc = tcp_get_socket (TCP_TYPE_SERVER, 0, 30, tcp_callback);
	if (tcp_soc != 0) {
		/* Start listening on TCP port 2000 */
		tcp_listen (tcp_soc, 2000);
	}
	
//	delay(5000);
	//printf("\n*** In The Name of ALLAH ***");
	SystemConfiguration();
//	printf("\nSystem Configuration Done");
//	printf("\nSystem Startup");
	
	//FlashOptionByteCRLock(False);
 	while(1){
 		/*if ((Rx_Desc[0].Stat & 0x80000000) == 0){
			for (i = 0 ; i < ((Rx_Desc[0].Stat >> 16) & 0x3FFF) ; i++)
				printf("%04X" , ((uint8*)(Rx_Desc[0].Addr))[i]);
		}*/
		main_TcpNet();
		
		timer++;
		if (timer == 35000){
			timer = 0;
			timer_tick();
		}

		if (SPI1 -> SR & SPI_SR_RXNE)
			DataRead = SPI1 -> DR;
		
		if (DataReceivedFlag == True){
			DataReceivedFlag = False;
			switch(((uint16*)TCPRxDataPtr)[0]){
				case 0x10:
					if (tcp_check_send (tcp_soc)) {
						sendbuf = tcp_get_buf (sizeof(Data));
						memcpy (sendbuf, Data, sizeof(Data));
						tcp_send (tcp_soc, sendbuf, sizeof(Data));
					}
					GPIOD -> ODR = (1 << 12);
				break;
				case 0x11:
					printf("\n*** In The Name of ALLAH ***");
					GPIOD -> ODR = (1 << 13);
				break;
				case 0x12:
					GPIOD -> ODR = (1 << 14);
				break;
				case 0x13:
					GPIOD -> ODR = (1 << 15);
				break;
			}
		}
	}
	//LED1();
}

/******************************************************************************************************/





