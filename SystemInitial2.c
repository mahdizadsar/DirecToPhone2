/*
#####################################################################################
#								In the name of ALLAH 								#
# Project : STM32F407 Test Project													#
# Date : 2013/11/18																	#
# file : SystemInitial.c 															#
#																					#
#####################################################################################
*/

// *** <<< Use Configuration Wizard in Context Menu >>> ***

#include "stm32f4xx.h"
#include "types.h"
#include "SystemInitial.h"
/******************************************************************************************************/

/*------------------------------------ Power Control Registers ---------------------------------------*/
//<h> Embedded Flash Memory Interface
//	<h> Flash Access Control Register (FLASH_ACR)
//		<q0.10> Data cache enable (DCEN)
//		<q0.9> Instruction cache enable (ICEN)
//		<q0.8> Prefetch enable (PRFTEN)
//		<o0.0..2> Latency (LATENCY)
//			<0=> 0 wait state
//			<1=> 1 wait state
//			<2=> 2 wait states
//			<3=> 3 wait states
//			<4=> 4 wait states
//			<5=> 5 wait states
//			<6=> 6 wait states
//			<7=> 7 wait states  
//	</h>

#define FLASH_ACR_VALUE				0x00000305

//	<h> Flash Control Register (FLASH_CR)
//		<q0.25> Error interrupt enable (ERRIE
// 		<q0.24> End of operation interrupt enable (EOPIE)
// 		<o0.8..9> Program size (PSIZE)
// 			<0=> Program x8
// 			<1=> Program x16
// 			<2=> Program x32
// 			<3=> Program x64
//	</h>

#define FLASH_CR_VALUE				0x00000200

//</h>



//<h> Power Controler (PWR)
//	<h> PWR power control register (PWR_CR)
//		<o0.14> Regulator voltage scaling output selection (VOS)
//			<0=> Scale 2 mode
//			<1=> Scale 1 mode (default value at reset)
//		<q0.9> Flash power-down in Stop mode (FPDS)
//		<q0.8> Disable backup domain write protection (DBP)
//		<i> 0: Access to RTC and RTC Backup registers and backup SRAM disabled
//		<i> 1: Access to RTC and RTC Backup registers and backup SRAM enabled
//		<o0.5..7> PVD level selection (PLS[2:0])
//			<0=> 2.0 V
//			<1=> 2.1 V
//			<2=> 2.3 V
//			<3=> 2.5 V
//			<4=> 2.6 V
//			<5=> 2.7 V
//			<6=> 2.8 V
//			<7=> 2.9 V
//		<q0.4> Power voltage detector enable (PVDE)
//		<q0.1> Power-down deepsleep (PDDS)
//		<i> 0: Enter Stop mode when the CPU enters deepsleep. The regulator status depends on the LPDS bit.
//		<i> 1: Enter Standby mode when the CPU enters deepsleep.
//		<q0.0> Low-power deepsleep (LPDS)
//		<i> 0: Voltage regulator on during Stop mode
//		<i> 1: Voltage regulator in low-power mode during Stop mode
//	</h>

#define PWR_CR_VALUE			0x00004100

//	<h> PWR Power Control/Status Register (PWR_CSR)
//		<q0.9> Backup regulator enable (BRE)
//		<i> When set, the Backup regulator (used to maintain backup SRAM content in Standby and
//		<i> VBAT modes) is enabled. If BRE is reset, the backup regulator is switched off. The backup
//		<i> SRAM can still be used but its content will be lost in the Standby and VBAT modes. Once set,
//		<i> the application must wait that the Backup Regulator Ready flag (BRR) is set to indicate that
//		<i> the data written into the RAM will be maintained in the Standby and VBAT modes.
//		<q0.8> Enable WKUP pin (EWUP)
//		<i> 0: WKUP pin is used for general purpose I/O. An event on the WKUP pin does not wakeup the device from Standby mode.
//		<i> 1: WKUP pin is used for wakeup from Standby mode and forced in input pull down configuration 
//		<i> (rising edge on WKUP pin wakes-up the system from Standby mode).
//	</h>
//</h>

#define PWR_CSR_VALUE			0x00000000

/*------------------------------- Reset and Clock Control Registers --------------------------------*/

//<h> Reset and Clock Control (RCC)
//	<h> Reset
//
//	</h>
//	<h> Clock

//		<h> Clock Control Register (RCC_CR)
//			<q0.28> PLLSAI Enable (PLLISAION)
//			<i> Set and cleared by software to enable PLLSAI.
//			<q0.26> PLLI2C Enable (PLLI2SON)
//			<i> Set and cleared by software to enable PLLI2S.
//			<q0.24> Main PLL (PLL) enable (PLLON)
//			<i> Set and cleared by software to enable PLL.
//			<q0.19> Clock security system enable (CSSON)
//			<q0.18> HSE clock bypass (HSEBYP)
//			<q0.16> HSE clock enable (HSEON)
//			<o0.3..7> Internal high-speed clock trimming (HSITRIM[4:0]) <0-31>
//			<q0.0> Internal high-speed clock enable (HSION)
//		</h>

#define RCC_CR_VALUE			0x01010001

//		<h> PLL Configuration Register (RCC_PLLCFGR)
//			<o0.24..27> Main PLL Division for USB OTG FS, SDIO and RNG (PLLQ) <2-15>
//			<i> These bits should be written only if PLL is disabled.
//			<o0.22> Main PLL(PLL) and audio PLL(PLLI2S) source clock (PLLSRC)
//				<0=> HSI Clock 
//				<1=> HSE Oscillator Clock 
//			<o0.16..17> Main PLL (PLL) Division for main system clock (PLLP) 
//				<0=>   Division by 2
//				<1=>   Division by 4
//				<2=>   Division by 6
//				<3=>   Division by 8
//			<o0.6..14> Main PLL (PLL) Multiplication for VCO (PLLN) <192-432>
//			<i> The software has to set these bits correctly to ensure that
// 			<i> the VCO output frequency is between 192 and 432 MHz
//			<o0.0..5>  Division for the main PLL (PLL) and audio PLL (PLLI2S) input clock (PLLM) <2-63>
//			<i> The software has to set these bits correctly to ensure that the VCO input frequency
//			<i> ranges from 1 to 2 MHz. It is recommended to select a frequency of 2 MHz to limit
//			<i> PLL jitter.
//		</h> 

#define RCC_PLLCFGR_VALUE		0x07404B08

//		<h> RCC Clock Configuration Register (RCC_CFGR)
//			<o0.30..31> Microcontroller Clock Output 2 (MCO2[1:0]) 
//				<0=> System Clock (SYSCLK) 
//				<1=> PLLI2S Clock
//				<2=> HSE Oscillator Clock
//				<3=> PLL Clock 
//			<o0.27..29> MCO2 Prescaler (MCO2PRE)
//				<0=> No Division
//				<4=> Division by 2
//				<5=> Division by 3
//				<6=> Division by 4
//				<7=> Division by 5
//			<o0.24..26> MCO1 Prescaler (MCO1PRE)
//				<0=> No Division
//				<4=> Division by 2
//				<5=> Division by 3
//				<6=> Division by 4
//				<7=> Division by 5
//			<o0.23> I2S Clock Source (I2SSRC)
//				<0=> PLLI2S Clock Clock 
//				<1=> I2S_CKIN Pin Clock 
//			<o0.21..22> Microcontroller clock output 1 (MCO1)
//				<0=> HSI Clock 
//				<1=> LSE Scillator 
//				<2=> HSE Oscillator Clock 
//				<3=> PLL Clock 
//			<o0.16..20> HSE Division Factor for RTC Clock (RTCPRE) <0-31>			
//			<i> The software has to set these bits correctly to ensure that the clock supplied to the
//			<i> RTC is 1 MHz. These bits must be configured if needed before selecting the RTC
//			<i> clock source (0 , 1 = No Clock)
//			<o0.13..15> APB high-speed prescaler (APB2) (PPRE2)
//				<0=> AHB clock not divided
//				<4=> AHB clock divided by 2
//				<5=> AHB clock divided by 4
//				<6=> AHB clock divided by 8
//				<7=> AHB clock divided by 16
//			<i> The software has to set these bits correctly not to exceed 84 MHz on this domain.
//			<i> The clocks are divided with the new prescaler factor from 1 to 16 AHB cycles after
//			<i> PPRE2 write.
//			<o0.10..12> APB Low speed prescaler (APB1) (PPRE1)
//				<0=> AHB clock not divided
//				<4=> AHB clock divided by 2
//				<5=> AHB clock divided by 4
//				<6=> AHB clock divided by 8
//				<7=> AHB clock divided by 16
//			<i> The software has to set these bits correctly not to exceed 42 MHz on this domain.
//			<i> The clocks are divided with the new prescaler factor from 1 to 16 AHB cycles after
//			<i> PPRE1 write.
//			<o0.4..7> AHB prescaler (HPRE)
//				<0=> system clock not divided
//				<8=> system clock divided by 2
//				<9=> system clock divided by 4
//				<10=> system clock divided by 8
//				<11=> system clock divided by 16
//				<12=> system clock divided by 64
//				<13=> system clock divided by 128
//				<14=> system clock divided by 256
//				<15=> system clock divided by 512
//			<i> The clocks are divided with the new prescaler factor from 1 to 16 AHB cycles after HPRE write.
//			<i> The AHB clock frequency must be at least 25 MHz when the Ethernet is used.
//			<o0.0..1> System clock switch (SW)
//				<0=> HSI Oscillator
//				<1=> HSE Oscillator
//				<2=> PLL 
//		</h>

#define RCC_CFGR_VALUE			0x3D609402

//		<h> RCC Clock Interrupt Register (RCC_CIR)	
//			<q0.13> PLLI2S ready interrupt enable (CPLLI2SRDYIE)
//			<q0.12> Main PLL (PLL) ready interrupt enable (PLLRDYIE)
//			<q0.11> HSE ready interrupt enable (HSERDYIE)
//			<q0.10> HSI ready interrupt enable (HSIRDYIE)
//			<q0.9> LSE ready interrupt enable (LSERDYIE)
//			<q0.8> LSI ready interrupt enable (LSIRDYIE)
//		</h>
#define RCC_CIR_VALUE			0x00000000

//		<h> RCC AHB1 Peripheral Clock Enable Register (RCC_AHB1ENR)
//			<q0.30> USB OTG HSULPI clock enable (OTGHSULPIEN)
//			<q0.29> USB OTG HS clock enable (OTGHSEN)
//			<q0.29> Ethernet PTP clock enable (ETHMACPTPEN)
//			<q0.27> Ethernet Reception clock enable (ETHMACRXEN)
//			<q0.26> Ethernet Transmission clock enable (ETHMACTXEN)
//			<q0.25> Ethernet MAC clock enable (ETHMACEN)
//			<q0.22> DMA2 clock enable (DMA2EN)
//			<q0.21> DMA1 clock enable (DMA1EN)
//			<q0.20> CCM data RAM clock enable (CCMDATARAMEN)
//			<q0.18> Backup SRAM interface clock enable (BKPSRAMEN)
//			<q0.12> CRC clock enable (CRCEN)
//			<q0.8> IO port I clock enable (GPIOIEN)
//			<q0.7> IO port H clock enable (GPIOHEN)
//			<q0.6> IO port G clock enable (GPIOGEN)
//			<q0.5> IO port F clock enable (GPIOFEN)
//			<q0.4> IO port E clock enable (GPIOEEN)
//			<q0.3> IO port D clock enable (GPIODEN)
//			<q0.2> IO port C clock enable (GPIOCEN)
//			<q0.1> IO port B clock enable (GPIOBEN)
//			<q0.0> IO port A clock enable (GPIOAEN)
//		</h>

#define RCC_AHB1ENR_VALUE		0x2E60011F

//		<h> RCC AHB2 Peripheral Clock Enable Register (RCC_AHB2ENR)
//			<q0.7> USB OTG FS clock enable (OTGFSEN)
//			<q0.6> Random number generator clock enable (RNGEN)
//			<q0.5> Hash modules clock enable (HASHEN)
//			<q0.4> Cryptographic modules clock enable (CRYPEN)
//			<q0.0> Camera interface enable (DCMIEN)
//		</h>

#define RCC_AHB2ENR_VALUE		0x00000000

//		<h>  RCC AHB3 Peripheral Clock Enable Register (RCC_AHB3ENR)
//			<q0.0> Flexible static memory controller module clock enable (FSMCEN)
//		</h>

#define RCC_AHB3ENR_VALUE		0x00000000

//		<h> RCC APB1 Peripheral Clock Enable Register (RCC_APB1ENR)
//			<q0.29> DAC interface clock enable (DACEN)
//			<q0.28> Power interface clock enable (PWREN)
//			<q0.26> CAN 2 clock enable (CAN2EN)
//			<q0.25> CAN 1 clock enable (CAN1EN)
//			<q0.23> I2C3 clock enable (I2C3EN)
//			<q0.22> I2C2 clock enable (I2C2EN)
//			<q0.21> I2C1 clock enable (I2C1EN)
//			<q0.20> UART5 clock enable (UART5EN)
//			<q0.19> UART4 clock enable (UART4EN)
//			<q0.18> USART3 clock enable (USART3EN)
//			<q0.17> USART2 clock enable (USART2EN)
//			<q0.15> SPI3 clock enable (SPI3EN)
//			<q0.14> SPI2 clock enable  (SPI2EN)
//			<q0.11> Window watchdog clock enable (WWDGEN)
//			<q0.8> TIM14 clock enable (TIM14EN)
//			<q0.7> TIM13 clock enable (TIM13EN)
//			<q0.6> TIM12 clock enable (TIM12EN)
//			<q0.5> TIM7 clock enable (TIM7EN)
//			<q0.4> TIM6 clock enable (TIM6EN)
//			<q0.3> TIM5 clock enable (TIM5EN)
//			<q0.2> TIM4 clock enable (TIM4EN)
//			<q0.1> TIM3 clock enable (TIM3EN)
//			<q0.0> TIM2 clock enable (TIM2EN)
//		</h>

#define RCC_APB1ENR_VALUE		0x10040000

//		<h> RCC APB2 Peripheral Clock Enable Register (RCC_APB2ENR)
//			<q0.21> SPI6 clock enable (SPI6EN)
//			<q0.20> SPI5 clock enable (SPI5EN)
//			<q0.18> TIM11 clock enable (TIM11EN)
//			<q0.17> TIM10 clock enable (TIM10EN)
//			<q0.16> TIM9 clock enable (TIM9EN)
//			<q0.14> System configuration controller clock enable (SYSCFGEN)
//			<q0.13> SPI4 clock enable (SPI4EN)
//			<q0.12> SPI1 clock enable (SPI1EN)
//			<q0.11> SDIO clock enable (SDIOEN)
//			<q0.10> ADC3 clock enable (ADC3EN)
//			<q0.9> ADC2 clock enable (ADC2EN)
//			<q0.8> ADC1 clock enable (ADC1EN)
//			<q0.5> USART6 clock enable (USART6EN)
//			<q0.4> USART1 clock enable (USART1EN)
//			<q0.1> TIM8 clock enable (TIM8EN)
//			<q0.0> TIM1 clock enable (TIM1EN)
//		</h>

#define RCC_APB2ENR_VALUE		0x00005010

//		<h> RCC AHB1 Peripheral Clock Enable in Low Power Mode Register (RCC_AHB1LPENR)
//			<q0.30> USB OTG HS ULPI clock enable during Sleep mode (OTGHSULPILPEN)
//			<q0.29> USB OTG HS clock enable during Sleep mode (OTGHSLPEN)
//			<q0.28> Ethernet PTP clock enable during Sleep mode (ETHMACPTPLPEN)
//			<q0.27> Ethernet reception clock enable during Sleep mode (ETHMACRXLPEN)
//			<q0.26> Ethernet transmission clock enable during Sleep mode (ETHMACTXLPEN)
//			<q0.25> Ethernet MAC clock enable during Sleep mode (ETHMACLPEN)
//			<q0.22> DMA2 clock enable during Sleep mode (DMA2LPEN)
//			<q0.21> DMA1 clock enable during Sleep mode (DMA1LPEN)
//			<q0.18> Backup SRAM interface clock enable during Sleep mode (BKPSRAMLPEN)
//			<q0.17> SRAM 2 interface clock enable during Sleep mode (SRAM2LPEN)
//			<q0.16> SRAM 1interface clock enable during Sleep mode (SRAM1LPEN)
//			<q0.15> Flash interface clock enable during Sleep mode (FLITFLPEN)
//			<q0.12> CRC clock enable during Sleep mode (CRCLPEN)
//			<q0.8> IO port I clock enable during Sleep mode (GPIOILPEN)
//			<q0.7> IO port H clock enable during Sleep mode (GPIOHLPEN)
//			<q0.6> IO port G clock enable during Sleep mode (GPIOGLPEN)
//			<q0.5> IO port F clock enable during Sleep mode (GPIOFLPEN)
//			<q0.4> IO port E clock enable during Sleep mode (GPIOELPEN)
//			<q0.3> IO port D clock enable during Sleep mode (GPIODLPEN)
//			<q0.2> IO port C clock enable during Sleep mode (GPIOCLPEN)
//			<q0.1> IO port B clock enable during Sleep mode (GPIOBLPEN)
//			<q0.0> IO port A clock enable during sleep mode (GPIOALPEN)
//		</h>

#define RCC_AHB1LPENR_VALUE		0x00000000

//		<h> RCC AHB2 Peripheral Clock Enable in Low Power Mode Register (RCC_AHB2LPENR)
//			<q0.7> USB OTG FS clock enable during Sleep mode (OTGFSLPEN)
//			<q0.6> Random number generator clock enable during Sleep mode (RNGLPEN)
//			<q0.5> Hash modules clock enable during Sleep mode (HASHLPEN)
//			<q0.4> Cryptography modules clock enable during Sleep mode (CRYPLPEN)
//			<q0.0> Camera interface enable during Sleep mode (DCMILPEN)
//		</h> 

#define RCC_AHB2LPENR_VALUE		0x00000000

//		<h> RCC AHB3 Peripheral Clock Enable in Low Power Mode Register (RCC_AHB3LPENR)
//			<q0.0> Flexible static memory controller module clock enable during Sleep mode (FSMCLPEN)
//		</h>

#define RCC_AHB3LPENR_VALUE		0x00000000

//		<h> RCC APB1 Peripheral Clock Enable in Low Power Mode Register (RCC_APB1LPENR)
//			<q0.29> DAC interface clock enable during Sleep mode (DACLPEN)
//			<q0.28> Power interface clock enable during Sleep mode (PWRLPEN)
//			<q0.26> CAN 2 clock enable during Sleep mode (CAN2LPEN)
//			<q0.25> CAN 1 clock enable during Sleep mode (CAN1LPEN)
//			<q0.23> I2C3 clock enable during Sleep mode (I2C3LPEN)
//			<q0.22> I2C2 clock enable during Sleep mode (I2C2LPEN)
//			<q0.21> I2C1 clock enable during Sleep mode (I2C1LPEN)
//			<q0.20> UART5 clock enable during Sleep mode (UART5LPEN)
//			<q0.19> UART4 clock enable during Sleep mode (UART4LPEN)
//			<q0.18> USART3 clock enable during Sleep mode (USART3LPEN)
//			<q0.17> USART2 clock enable during Sleep mode (USART2LPEN)
//			<q0.15> SPI3 clock enable during Sleep mode (SPI3LPEN)
//			<q0.14> SPI2 clock enable during Sleep mode (SPI2LPEN)
//			<q0.11> Window watchdog clock enable during Sleep mode (WWDGLPEN)
//			<q0.8> TIM14 clock enable during Sleep mode (TIM14LPEN)
//			<q0.7> TIM13 clock enable during Sleep mode (TIM13LPEN)
//			<q0.6> TIM12 clock enable during Sleep mode (TIM12LPEN)
//			<q0.5> TIM7 clock enable during Sleep mode (TIM7LPEN)
//			<q0.4> TIM6 clock enable during Sleep mode (TIM6LPEN)
//			<q0.3> TIM5 clock enable during Sleep mode (TIM5LPEN)
//			<q0.2> TIM4 clock enable during Sleep mode (TIM4LPEN)
//			<q0.1> TIM3 clock enable during Sleep mode (TIM3LPEN)
//			<q0.0> TIM2 clock enable during Sleep mode (TIM2LPEN)
//		</h>

#define RCC_APB1LPENR_VALUE		0x00000000

//		<h> RCC APB2 Peripheral Clock Enabled in Low Power Mode Register (RCC_APB2LPENR)
//			<q0.18> TIM11 clock enable during Sleep mode (TIM11LPEN)
//			<q0.17> TIM10 clock enable during Sleep mode (TIM10LPEN)
//			<q0.16> TIM9 clock enable during sleep mode (TIM9LPEN)
//			<q0.14> System configuration controller clock enable during Sleep mode (SYSCFGLPEN)
//			<q0.12> SPI1 clock enable during Sleep mode (SPI1LPEN)
//			<q0.11> SDIO clock enable during Sleep mode (SDIOLPEN)
//			<q0.10> ADC 3 clock enable during Sleep mode (ADC3LPEN)
//			<q0.9> ADC2 clock enable during Sleep mode (ADC2LPEN)
//			<q0.8> ADC1 clock enable during Sleep mode (ADC1LPEN)
//			<q0.5> USART6 clock enable during Sleep mode (USART6LPEN)
//			<q0.4> USART1 clock enable during Sleep mode (USART1LPEN)
//			<q0.1> TIM8 clock enable during Sleep mode (TIM8LPEN)
//			<q0.0> TIM1 clock enable during Sleep mode (TIM1LPEN)
//		</h>

#define RCC_APB2LPENR_VALUE		0x00000000

//		<h> RCC Backup Domain Control Register (RCC_BDCR)
//			<o0.16> Backup domain software reset (BDRST)
//			<i> The LSEON, LSEBYP, RTCSEL and RTCEN bits in the RCC Backup domain control register (RCC_BDCR) are in the Backup domain. As a result, after Reset, these bits are
//			<i> write-protected and the DBP bit in the PWR power control register (PWR_CR) for STM32F405xx/07xx and STM32F415xx/17xx has to be set before these can be modified.
//			<o0.15> RTC clock enable (RTCEN)
//			<i> The LSEON, LSEBYP, RTCSEL and RTCEN bits in the RCC Backup domain control register (RCC_BDCR) are in the Backup domain. As a result, after Reset, these bits are
//			<i> write-protected and the DBP bit in the PWR power control register (PWR_CR) for STM32F405xx/07xx and STM32F415xx/17xx has to be set before these can be modified.
//			<o0.8..9> RTC clock source selection (RTCSEL[1:0])
//				<0=> No clock
//				<1=> LSE Clock 
//				<2=> LSI Clock 
//				<3=> HSE Clock DIV by RTCPRE
//			<i> HSE oscillator clock divided by a programmable prescaler (selection through the RTCPRE[4:0] bits in the RCC clock configuration register (RCC_CFGR)) used as the RTC clock
//			<i> .
//			<i> The LSEON, LSEBYP, RTCSEL and RTCEN bits in the RCC Backup domain control register (RCC_BDCR) are in the Backup domain. As a result, after Reset, these bits are
//			<i> write-protected and the DBP bit in the PWR power control register (PWR_CR) for STM32F405xx/07xx and STM32F415xx/17xx has to be set before these can be modified.
//			<o0.2> External low-speed oscillator bypass (LSEBYP)
//			<i> The LSEON, LSEBYP, RTCSEL and RTCEN bits in the RCC Backup domain control register (RCC_BDCR) are in the Backup domain. As a result, after Reset, these bits are
//			<i> write-protected and the DBP bit in the PWR power control register (PWR_CR) for STM32F405xx/07xx and STM32F415xx/17xx has to be set before these can be modified.
//			<o0.1> External low-speed oscillator ready (LSERDY)
//			<i> The LSEON, LSEBYP, RTCSEL and RTCEN bits in the RCC Backup domain control register (RCC_BDCR) are in the Backup domain. As a result, after Reset, these bits are
//			<i> write-protected and the DBP bit in the PWR power control register (PWR_CR) for STM32F405xx/07xx and STM32F415xx/17xx has to be set before these can be modified.
//			<o0.0> External low-speed oscillator enable (LSEON)
//			<i> The LSEON, LSEBYP, RTCSEL and RTCEN bits in the RCC Backup domain control register (RCC_BDCR) are in the Backup domain. As a result, after Reset, these bits are
//			<i> write-protected and the DBP bit in the PWR power control register (PWR_CR) for STM32F405xx/07xx and STM32F415xx/17xx has to be set before these can be modified.
//		</h>

#define RCC_BDCR_VALUE			0x00000000

//		<h> RCC Clock Control & Status Register (RCC_CSR)
//			<o0.0> Internal low-speed oscillator enable (LSION)
//		</h>

#define RCC_CSR_VALUE 			0x00000000

//		<h> RCC PLLI2S Configuration Register (RCC_PLLI2SCFGR)
//			<o0.28..30> PLLI2S division factor for I2S clocks (PLLI2SR) <2-7>
//			<o0.6..14> PLLI2S multiplication factor for VCO (PLLI2SN) <192-432>
//		</h>
//	</h> 
//</h> 

#define RCC_PLLI2SCFGR_VALUE	0x00000000

/*------------------------------------------ GPIO Registers -----------------------------------------*/
//<h> General Purpose I/O (GPIO)
//	<h> GPIO Port Mode Register (GPIOx_MODER)
//		<h> GPIO PORTA
//			<o0.0..1> PORT0
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.2..3> PORT1
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.4..5> PORT2
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.6..7> PORT3
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.8..9> PORT4
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.10..11> PORT5
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.12..13> PORT6
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.14..15> PORT7
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.16..17> PORT8
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.18..19> PORT9
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.20..21> PORT10
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.22..23> PORT11
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.24..25> PORT12
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.26..27> PORT13
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.28..29> PORT14
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.30..31> PORT15
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//		</h> GPIO PORTA

#define GPIOA_MODER_VALUE	0xA80280AA

//		<h> GPIO PORTB
//			<o0.0..1> PORT0
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.2..3> PORT1
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.4..5> PORT2
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.6..7> PORT3
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.8..9> PORT4
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.10..11> PORT5
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.12..13> PORT6
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.14..15> PORT7
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.16..17> PORT8
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.18..19> PORT9
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.20..21> PORT10
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.22..23> PORT11
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.24..25> PORT12
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.26..27> PORT13
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.28..29> PORT14
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.30..31> PORT15
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//		</h> GPIO PORTB

#define GPIOB_MODER_VALUE	0x0AA22A8A

//		<h> GPIO PORTC
//			<o0.0..1> PORT0
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.2..3> PORT1
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.4..5> PORT2
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.6..7> PORT3
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.8..9> PORT4
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.10..11> PORT5
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.12..13> PORT6
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.14..15> PORT7
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.16..17> PORT8
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.18..19> PORT9
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.20..21> PORT10
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.22..23> PORT11
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.24..25> PORT12
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.26..27> PORT13
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.28..29> PORT14
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.30..31> PORT15
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//		</h> GPIO PORTC

#define GPIOC_MODER_VALUE	0x00011AA8

//		<h> GPIO PORTD
//			<o0.0..1> PORT0
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.2..3> PORT1
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.4..5> PORT2
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.6..7> PORT3
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.8..9> PORT4
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.10..11> PORT5
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.12..13> PORT6
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.14..15> PORT7
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.16..17> PORT8
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.18..19> PORT9
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.20..21> PORT10
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.22..23> PORT11
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.24..25> PORT12
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.26..27> PORT13
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.28..29> PORT14
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.30..31> PORT15
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//		</h> GPIO PORTD

#define GPIOD_MODER_VALUE	0x551A0400

//		<h> GPIO PORTE
//			<o0.0..1> PORT0
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.2..3> PORT1
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.4..5> PORT2
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.6..7> PORT3
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.8..9> PORT4
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.10..11> PORT5
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.12..13> PORT6
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.14..15> PORT7
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.16..17> PORT8
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.18..19> PORT9
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.20..21> PORT10
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.22..23> PORT11
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.24..25> PORT12
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.26..27> PORT13
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.28..29> PORT14
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.30..31> PORT15
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//		</h> GPIO PORTE

#define GPIOE_MODER_VALUE	0x15504000

//		<h> GPIO PORTF
//			<o0.0..1> PORT0
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.2..3> PORT1
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.4..5> PORT2
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.6..7> PORT3
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.8..9> PORT4
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.10..11> PORT5
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.12..13> PORT6
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.14..15> PORT7
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.16..17> PORT8
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.18..19> PORT9
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.20..21> PORT10
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.22..23> PORT11
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.24..25> PORT12
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.26..27> PORT13
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.28..29> PORT14
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.30..31> PORT15
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//		</h> GPIO PORTF

#define GPIOF_MODER_VALUE	0x00000000

//		<h> GPIO PORTG
//			<o0.0..1> PORT0
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.2..3> PORT1
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.4..5> PORT2
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.6..7> PORT3
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.8..9> PORT4
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.10..11> PORT5
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.12..13> PORT6
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.14..15> PORT7
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.16..17> PORT8
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.18..19> PORT9
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.20..21> PORT10
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.22..23> PORT11
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.24..25> PORT12
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.26..27> PORT13
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.28..29> PORT14
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.30..31> PORT15
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//		</h> GPIO PORTG

#define GPIOG_MODER_VALUE	0x00000000

//		<h> GPIO PORTH
//			<o0.0..1> PORT0
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.2..3> PORT1
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.4..5> PORT2
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.6..7> PORT3
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.8..9> PORT4
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.10..11> PORT5
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.12..13> PORT6
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.14..15> PORT7
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.16..17> PORT8
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.18..19> PORT9
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.20..21> PORT10
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.22..23> PORT11
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.24..25> PORT12
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.26..27> PORT13
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.28..29> PORT14
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.30..31> PORT15
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//		</h> GPIO PORTH

#define GPIOH_MODER_VALUE	0x00000000

//		<h> GPIO PORTI
//			<o0.0..1> PORT0
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.2..3> PORT1
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.4..5> PORT2
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.6..7> PORT3
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.8..9> PORT4
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.10..11> PORT5
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.12..13> PORT6
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.14..15> PORT7
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.16..17> PORT8
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.18..19> PORT9
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.20..21> PORT10
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//			<o0.22..23> PORT11
//				<0=> Input
//				<1=> General Purpose Output
//				<2=> Alternate Function
//				<3=> Analog Mode
//		</h> GPIO PORTI

#define GPIOI_MODER_VALUE	0x00000000

//	</h> GPIO port mode register (GPIOx_MODER)

// 	<h> GPIO port output type register (GPIOx_OTYPER)
//		<h> GPIO PORTA
//			<o0.0> PORT0 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.1> PORT1 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.2> PORT2 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.3> PORT3 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.4> PORT4 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.5> PORT5 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.6> PORT6 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.7> PORT7
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.8> PORT8 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.9> PORT9 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.10> PORT10 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.11> PORT11 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.12> PORT12
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.13> PORT13 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.14> PORT14 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.15> PORT15 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//		</h>

#define GPIOA_OTYPER_VALUE	0x00000000

//		<h> GPIO PORTB
//			<o0.0> PORT0 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.1> PORT1 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.2> PORT2 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.3> PORT3 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.4> PORT4 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.5> PORT5 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.6> PORT6 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.7> PORT7
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.8> PORT8 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.9> PORT9 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.10> PORT10 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.11> PORT11 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.12> PORT12
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.13> PORT13 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.14> PORT14 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.15> PORT15 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//		</h>

#define GPIOB_OTYPER_VALUE	0x00000000

//		<h> GPIO PORTC
//			<o0.0> PORT0 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.1> PORT1 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.2> PORT2 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.3> PORT3 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.4> PORT4 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.5> PORT5 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.6> PORT6 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.7> PORT7
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.8> PORT8 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.9> PORT9 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.10> PORT10 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.11> PORT11 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.12> PORT12
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.13> PORT13 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.14> PORT14 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.15> PORT15 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//		</h>

#define GPIOC_OTYPER_VALUE	0x00000000

//		<h> GPIO PORTD
//			<o0.0> PORT0 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.1> PORT1 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.2> PORT2 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.3> PORT3 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.4> PORT4 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.5> PORT5 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.6> PORT6 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.7> PORT7
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.8> PORT8 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.9> PORT9 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.10> PORT10 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.11> PORT11 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.12> PORT12
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.13> PORT13 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.14> PORT14 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.15> PORT15 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//		</h>

#define GPIOD_OTYPER_VALUE	0x00000000

//		<h> GPIO PORTE
//			<o0.0> PORT0 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.1> PORT1 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.2> PORT2 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.3> PORT3 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.4> PORT4 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.5> PORT5 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.6> PORT6 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.7> PORT7
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.8> PORT8 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.9> PORT9 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.10> PORT10 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.11> PORT11 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.12> PORT12
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.13> PORT13 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.14> PORT14 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.15> PORT15 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//		</h>

#define GPIOE_OTYPER_VALUE	0x00000000

//		<h> GPIO PORTF
//			<o0.0> PORT0 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.1> PORT1 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.2> PORT2 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.3> PORT3 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.4> PORT4 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.5> PORT5 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.6> PORT6 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.7> PORT7
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.8> PORT8 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.9> PORT9 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.10> PORT10 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.11> PORT11 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.12> PORT12
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.13> PORT13 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.14> PORT14 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.15> PORT15 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//		</h>

#define GPIOF_OTYPER_VALUE	0x00000000

//		<h> GPIO PORTG
//			<o0.0> PORT0 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.1> PORT1 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.2> PORT2 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.3> PORT3 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.4> PORT4 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.5> PORT5 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.6> PORT6 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.7> PORT7
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.8> PORT8 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.9> PORT9 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.10> PORT10 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.11> PORT11 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.12> PORT12
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.13> PORT13 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.14> PORT14 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.15> PORT15 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//		</h>

#define GPIOG_OTYPER_VALUE	0x00000000

//		<h> GPIO PORTH
//			<o0.0> PORT0 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.1> PORT1 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.2> PORT2 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.3> PORT3 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.4> PORT4 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.5> PORT5 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.6> PORT6 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.7> PORT7
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.8> PORT8 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.9> PORT9 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.10> PORT10 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.11> PORT11 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.12> PORT12
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.13> PORT13 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.14> PORT14 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.15> PORT15 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//		</h>

#define GPIOH_OTYPER_VALUE	0x00000000

//		<h> GPIO PORTI
//			<o0.0> PORT0 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.1> PORT1 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.2> PORT2 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.3> PORT3 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.4> PORT4 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.5> PORT5 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.6> PORT6 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.7> PORT7
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.8> PORT8 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.9> PORT9 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.10> PORT10 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//			<o0.11> PORT11 
//				<0=> Output Push-Pull
//				<1=> Output Open-Drain
//		</h>

#define GPIOI_OTYPER_VALUE	0x00000000

//	</h> GPIO port output type register (GPIOx_OTYPER)


//	<h> GPIO Port Output Speed Register (GPIOx_OSPEEDR)
//		<h> GPIO PORTA 
//			<o0.0..1> PORT0
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.2..3> PORT1
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.4..5> PORT2
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.6..7> PORT3
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.8..9> PORT4
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.10..11> PORT5
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.12..13> PORT6
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.14..15> PORT7
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.16..17> PORT8
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.18..19> PORT9
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.20..21> PORT10
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.22..23> PORT11
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.24..25> PORT12
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.26..27> PORT13
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.28..29> PORT14
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.30..31> PORT15
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//		</h> GPIO PORTA 

#define GPIOA_OSPEEDR_VALUE	0x0000808A

//		<h> GPIO PORTB 
//			<o0.0..1> PORT0
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.2..3> PORT1
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.4..5> PORT2
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.6..7> PORT3
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.8..9> PORT4
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.10..11> PORT5
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.12..13> PORT6
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.14..15> PORT7
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.16..17> PORT8
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.18..19> PORT9
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.20..21> PORT10
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.22..23> PORT11
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.24..25> PORT12
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.26..27> PORT13
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.28..29> PORT14
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.30..31> PORT15
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//		</h> GPIO PORTB 

#define GPIOB_OSPEEDR_VALUE	0x0AA20A8A

//		<h> GPIO PORTC 
//			<o0.0..1> PORT0
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.2..3> PORT1
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.4..5> PORT2
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.6..7> PORT3
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.8..9> PORT4
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.10..11> PORT5
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.12..13> PORT6
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.14..15> PORT7
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.16..17> PORT8
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.18..19> PORT9
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.20..21> PORT10
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.22..23> PORT11
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.24..25> PORT12
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.26..27> PORT13
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.28..29> PORT14
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.30..31> PORT15
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//		</h> GPIO PORTC 

#define GPIOC_OSPEEDR_VALUE	0x00000AA0

//		<h> GPIO PORTD 
//			<o0.0..1> PORT0
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.2..3> PORT1
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.4..5> PORT2
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.6..7> PORT3
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.8..9> PORT4
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.10..11> PORT5
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.12..13> PORT6
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.14..15> PORT7
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.16..17> PORT8
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.18..19> PORT9
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.20..21> PORT10
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.22..23> PORT11
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.24..25> PORT12
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.26..27> PORT13
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.28..29> PORT14
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.30..31> PORT15
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//		</h> GPIO PORTD 

#define GPIOD_OSPEEDR_VALUE	0x55060000

//		<h> GPIO PORTE 
//			<o0.0..1> PORT0
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.2..3> PORT1
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.4..5> PORT2
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.6..7> PORT3
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.8..9> PORT4
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.10..11> PORT5
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.12..13> PORT6
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.14..15> PORT7
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.16..17> PORT8
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.18..19> PORT9
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.20..21> PORT10
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.22..23> PORT11
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.24..25> PORT12
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.26..27> PORT13
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.28..29> PORT14
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.30..31> PORT15
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//		</h> GPIO PORTE 

#define GPIOE_OSPEEDR_VALUE	0x00000000


//		<h> GPIO PORTF 
//			<o0.0..1> PORT0
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.2..3> PORT1
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.4..5> PORT2
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.6..7> PORT3
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.8..9> PORT4
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.10..11> PORT5
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.12..13> PORT6
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.14..15> PORT7
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.16..17> PORT8
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.18..19> PORT9
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.20..21> PORT10
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.22..23> PORT11
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.24..25> PORT12
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.26..27> PORT13
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.28..29> PORT14
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.30..31> PORT15
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//		</h> GPIO PORTF 

#define GPIOF_OSPEEDR_VALUE	0x00000000

//		<h> GPIO PORTG 
//			<o0.0..1> PORT0
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.2..3> PORT1
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.4..5> PORT2
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.6..7> PORT3
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.8..9> PORT4
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.10..11> PORT5
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.12..13> PORT6
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.14..15> PORT7
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.16..17> PORT8
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.18..19> PORT9
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.20..21> PORT10
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.22..23> PORT11
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.24..25> PORT12
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.26..27> PORT13
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.28..29> PORT14
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.30..31> PORT15
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//		</h> GPIO PORTG 

#define GPIOG_OSPEEDR_VALUE	0x00000000


//		<h> GPIO PORTH 
//			<o0.0..1> PORT0
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.2..3> PORT1
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.4..5> PORT2
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.6..7> PORT3
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.8..9> PORT4
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.10..11> PORT5
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.12..13> PORT6
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.14..15> PORT7
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.16..17> PORT8
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.18..19> PORT9
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.20..21> PORT10
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.22..23> PORT11
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.24..25> PORT12
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.26..27> PORT13
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.28..29> PORT14
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.30..31> PORT15
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//		</h> GPIO PORTH 

#define GPIOH_OSPEEDR_VALUE	0x00000000


//		<h> GPIO PORTI 
//			<o0.0..1> PORT0
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.2..3> PORT1
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.4..5> PORT2
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.6..7> PORT3
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.8..9> PORT4
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.10..11> PORT5
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.12..13> PORT6
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.14..15> PORT7
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.16..17> PORT8
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.18..19> PORT9
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.20..21> PORT10
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//			<o0.22..23> PORT11
//				<0=> Low Speed
//				<1=> Medium Speed
//				<2=> Fast Speed
//				<3=> High Speed
//		</h> GPIO PORTI 

#define GPIOI_OSPEEDR_VALUE	0x00000000

//	</h> GPIO Port Output Speed Register (GPIOx_OSPEEDR)

// <h> GPIO port Pull-up/Pull-down Register (GPIOx_PUPDR)
//		<h> GPIO PORTA
//			<o0.0..1> PORT0
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.2..3> PORT1
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.4..5> PORT2
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.6..7> PORT3
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.8..9> PORT4
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.10..11> PORT5
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.12..13> PORT6
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.14..15> PORT7
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.16..17> PORT8
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.18..19> PORT9
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.20..21> PORT10
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.22..23> PORT11
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.24..25> PORT12
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.26..27> PORT13
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.28..29> PORT14
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.30..31> PORT15
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//		</h>

#define GPIOA_PUPDR_VALUE	0x00000000

//		<h> GPIO PORTB
//			<o0.0..1> PORT0
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.2..3> PORT1
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.4..5> PORT2
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.6..7> PORT3
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.8..9> PORT4
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.10..11> PORT5
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.12..13> PORT6
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.14..15> PORT7
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.16..17> PORT8
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.18..19> PORT9
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.20..21> PORT10
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.22..23> PORT11
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.24..25> PORT12
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.26..27> PORT13
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.28..29> PORT14
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.30..31> PORT15
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//		</h>

#define GPIOB_PUPDR_VALUE	0x00000000

//		<h> GPIO PORTC
//			<o0.0..1> PORT0
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.2..3> PORT1
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.4..5> PORT2
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.6..7> PORT3
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.8..9> PORT4
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.10..11> PORT5
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.12..13> PORT6
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.14..15> PORT7
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.16..17> PORT8
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.18..19> PORT9
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.20..21> PORT10
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.22..23> PORT11
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.24..25> PORT12
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.26..27> PORT13
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.28..29> PORT14
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.30..31> PORT15
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//		</h>

#define GPIOC_PUPDR_VALUE	0x00000000

//		<h> GPIO PORTD
//			<o0.0..1> PORT0
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.2..3> PORT1
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.4..5> PORT2
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.6..7> PORT3
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.8..9> PORT4
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.10..11> PORT5
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.12..13> PORT6
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.14..15> PORT7
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.16..17> PORT8
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.18..19> PORT9
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.20..21> PORT10
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.22..23> PORT11
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.24..25> PORT12
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.26..27> PORT13
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.28..29> PORT14
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.30..31> PORT15
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//		</h>

#define GPIOD_PUPDR_VALUE	0x00000000

//		<h> GPIO PORTE
//			<o0.0..1> PORT0
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.2..3> PORT1
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.4..5> PORT2
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.6..7> PORT3
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.8..9> PORT4
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.10..11> PORT5
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.12..13> PORT6
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.14..15> PORT7
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.16..17> PORT8
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.18..19> PORT9
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.20..21> PORT10
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.22..23> PORT11
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.24..25> PORT12
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.26..27> PORT13
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.28..29> PORT14
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.30..31> PORT15
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//		</h>

#define GPIOE_PUPDR_VALUE	0x00000000

//		<h> GPIO PORTF
//			<o0.0..1> PORT0
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.2..3> PORT1
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.4..5> PORT2
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.6..7> PORT3
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.8..9> PORT4
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.10..11> PORT5
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.12..13> PORT6
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.14..15> PORT7
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.16..17> PORT8
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.18..19> PORT9
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.20..21> PORT10
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.22..23> PORT11
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.24..25> PORT12
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.26..27> PORT13
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.28..29> PORT14
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.30..31> PORT15
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//		</h>

#define GPIOF_PUPDR_VALUE	0x00000000

//		<h> GPIO PORTG
//			<o0.0..1> PORT0
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.2..3> PORT1
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.4..5> PORT2
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.6..7> PORT3
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.8..9> PORT4
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.10..11> PORT5
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.12..13> PORT6
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.14..15> PORT7
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.16..17> PORT8
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.18..19> PORT9
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.20..21> PORT10
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.22..23> PORT11
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.24..25> PORT12
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.26..27> PORT13
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.28..29> PORT14
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.30..31> PORT15
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//		</h>

#define GPIOG_PUPDR_VALUE	0x00000000

//		<h> GPIO PORTH
//			<o0.0..1> PORT0
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.2..3> PORT1
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.4..5> PORT2
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.6..7> PORT3
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.8..9> PORT4
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.10..11> PORT5
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.12..13> PORT6
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.14..15> PORT7
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.16..17> PORT8
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.18..19> PORT9
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.20..21> PORT10
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.22..23> PORT11
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.24..25> PORT12
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.26..27> PORT13
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.28..29> PORT14
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.30..31> PORT15
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//		</h>

#define GPIOH_PUPDR_VALUE	0x00000000

//		<h> GPIO PORTI
//			<o0.0..1> PORT0
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.2..3> PORT1
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.4..5> PORT2
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.6..7> PORT3
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.8..9> PORT4
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.10..11> PORT5
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.12..13> PORT6
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.14..15> PORT7
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.16..17> PORT8
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.18..19> PORT9
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.20..21> PORT10
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//			<o0.22..23> PORT11
//				<0=> No Pull-up, Pull-down
//				<1=> Pull-up
//				<2=> Pull-down
//		</h>

#define GPIOI_PUPDR_VALUE	0x00000000

// 	</h> GPIO port Pull-up/Pull-down Register (GPIOx_PUPDR)


//	<h> GPIO Alternate Function Low/High Register (GPIOx_AFRL/GPIOx_AFRH)
//		<h> GPIO PORTA
//			<o0.0..3> PORT0
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM2_CH1_ETR)
//				<2=> AF2 (TIM 5_CH1)
//				<3=> AF3 (TIM8_ETR)
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART2_CTS)
//				<8=> AF8 (UART4_TX)
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 (ETH_MII_CRS)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT1
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM2_CH2)
//				<2=> AF2 (TIM5_CH2)
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART2_RTS)
//				<8=> AF8 (UART4_RX)
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 (ETH_MII_RX_CLK , ETH_RMII_REF_CLK)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT2
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM2_CH3)
//				<2=> AF2 (TIM5_CH3)
//				<3=> AF3 (TIM9_CH1)
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART2_TX)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 (ETH_MDIO)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT3
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM2_CH4)
//				<2=> AF2 (TIM5_CH4)
//				<3=> AF3 (TIM9_CH2)
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART2_RX)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_HS_ULPI_D0)
//				<11=> AF11 (ETH_MII_COL)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT4
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 (SPI1_NSS)
//				<6=> AF6 (SPI3_NSS , I2S3_WS)
//				<7=> AF7 (USART2_CK)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (OTG_HS_SOF)
//				<13=> AF13 (DCMI_HSYNC)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT5
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM2_CH1_ETR)
//				<2=> AF2 
//				<3=> AF3 (TIM8_CH1N)
//				<4=> AF4
//				<5=> AF5 (SPI1_SCK)
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_HS_ULPI_CK)
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT6
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_BKIN)
//				<2=> AF2 (TIM3_CH1)
//				<3=> AF3 (TIM8_BKIN)
//				<4=> AF4
//				<5=> AF5 (SPI1_MISO)
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (TIM13_CH1)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_PIXCK)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT7
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH1N)
//				<2=> AF2 (TIM3_CH2)
//				<3=> AF3 (TIM8_CH1N)
//				<4=> AF4
//				<5=> AF5 (SPI1_MOSI)
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (TIM14_CH1)
//				<10=> AF10
//				<11=> AF11 (ETH_MII_RX_DV , ETH_RMII_CRS_DV)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)


#define GPIOA_AFRL_VALUE	0xB000BBBB

//			<o0.0..3> PORT8
//				<0=> AF0 (MCO1)
//				<1=> AF1 (TIM1_CH1)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 (I2C3_SCL)
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART1_CK)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_FS_SOF)
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT9
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH2)
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 (I2C3_SMBA)
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART1_TX)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D0)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT10
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH3)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART1_RX)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_FS_ID)
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D1)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT11
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH4)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART1_CTS)
//				<8=> AF8 
//				<9=> AF9 (CAN1_RX)
//				<10=> AF10 (OTG_FS_DM)
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT12
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_ETR)
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 (USART1_RTS)
//				<8=> AF8 
//				<9=> AF9 (CAN1_TX)
//				<10=> AF10 (OTG_FS_DP)
//				<11=> AF11 
//				<12=> AF12 
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT13
//				<0=> AF0 (JTMS , SWDIO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT14
//				<0=> AF0 (JTCK , SWCLK)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT15
//				<0=> AF0 (JTDI)
//				<1=> AF1 (TIM 2_CH1 , TIM 2_ETR)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 (SPI1_NSS)
//				<6=> AF6 (SPI3_NSS , I2S3_WS)
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//		</h>

#define GPIOA_AFRH_VALUE	0x50000000

//		<h> GPIO PORTB
//			<o0.0..3> PORT0
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH2N)
//				<2=> AF2 (TIM3_CH3)
//				<3=> AF3 (TIM8_CH2N)
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_HS_ULPI_D1)
//				<11=> AF11 (ETH _MII_RXD2)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT1
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH3N)
//				<2=> AF2 (TIM3_CH4)
//				<3=> AF3 (TIM8_CH3N)
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_HS_ULPI_D2)
//				<11=> AF11 (ETH _MII_RXD3)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT2
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT3
//				<0=> AF0 (JTDO/TRACES/WO)
//				<1=> AF1 (TIM2_CH2)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 (SPI1_SCK)
//				<6=> AF6 (SPI3_SCK , I2S3_CK)
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT4
//				<0=> AF0 (NJTRST)
//				<1=> AF1 
//				<2=> AF2 (TIM3_CH1)
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 (SPI1_MISO)
//				<6=> AF6 (SPI3_MISO)
//				<7=> AF7 (I2S3ext_SD)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT5
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM3_CH2)
//				<3=> AF3 
//				<4=> AF4 (I2C1_SMBA)
//				<5=> AF5 (SPI1_MOSI)
//				<6=> AF6 (SPI3_MOSI , I2S3_SD)
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (CAN2_RX)
//				<10=> AF10 (OTG_HS_ULPI_D7)
//				<11=> AF11 (ETH _PPS_OUT)
//				<12=> AF12
//				<13=> AF13 (DCMI_D10)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT6
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM4_CH1)
//				<3=> AF3 
//				<4=> AF4 (I2C1_SCL)
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 (USART1_TX)
//				<8=> AF8 
//				<9=> AF9 (CAN2_TX)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D5)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT7
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM4_CH2)
//				<3=> AF3 
//				<4=> AF4 (I2C1_SDA)
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 (USART1_RX)
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_NL)
//				<13=> AF13 (DCMI_VSYNC)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)


#define GPIOB_AFRL_VALUE	0x075550BB

//			<o0.0..3> PORT8
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM4_CH3)
//				<3=> AF3 (TIM10_CH1)
//				<4=> AF4 (I2C1_SCL)
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (CAN1_RX)
//				<10=> AF10  
//				<11=> AF11 (ETH _MII_TXD3)
//				<12=> AF12 (SDIO_D4)
//				<13=> AF13 (DCMI_D6)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT9
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM4_CH4)
//				<3=> AF3 (TIM11_CH1)
//				<4=> AF4 (I2C1_SDA)
//				<5=> AF5 (SPI2_NSS , I2S2_WS)
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (CAN1_TX)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (SDIO_D5)
//				<13=> AF13 (DCMI_D7)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT10
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM2_CH3)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 (I2C2_SCL)
//				<5=> AF5 (SPI2_SCK , I2S2_CK)
//				<6=> AF6
//				<7=> AF7 (USART3_TX)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_HS_ULPI_D3)
//				<11=> AF11 (ETH_MII_RX_ER)
//				<12=> AF12
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT11
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM2_CH4)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 (I2C2_SDA)
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART3_RX)
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10 (OTG_HS_ULPI_D4)
//				<11=> AF11 (ETH_MII_TX_EN , ETH_RMII_TX_EN)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT12
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_BKIN)
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 (I2C2_SMBA)
//				<5=> AF5 (SPI2_NSS , I2S2_WS)
//				<6=> AF6 
//				<7=> AF7 (USART3_CK)
//				<8=> AF8 
//				<9=> AF9 (CAN2_RX)
//				<10=> AF10 (OTG_HS_ULPI_D5)
//				<11=> AF11 (ETH_MII_TXD0 , ETH_RMII_TXD0)
//				<12=> AF12 (OTG_HS_ID)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT13
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH1N)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 (SPI2_SCK , I2S2_CK)
//				<6=> AF6
//				<7=> AF7 (USART3_CTS)
//				<8=> AF8 
//				<9=> AF9 (CAN2_TX)
//				<10=> AF10 (OTG_HS_ULPI_D6)
//				<11=> AF11 (ETH_MII_TXD1 , ETH_RMII_TXD1)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT14
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH2N)
//				<2=> AF2 
//				<3=> AF3 (TIM8_CH2N)
//				<4=> AF4
//				<5=> AF5 (SPI2_MISO)
//				<6=> AF6 (I2S2ext_SD)
//				<7=> AF7 (USART3_RTS)
//				<8=> AF8 
//				<9=> AF9 (TIM12_CH1)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (OTG_HS_DM)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT15
//				<0=> AF0 (RTC_REFIN)
//				<1=> AF1 (TIM1_CH3N)
//				<2=> AF2 
//				<3=> AF3 (TIM8_CH3N)
//				<4=> AF4
//				<5=> AF5 (SPI2_MOSI , I2S2_SD)
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (TIM12_CH2)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (OTG_HS_DP)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//		</h>

#define GPIOB_AFRH_VALUE	0x00BBBB0B


//		<h> GPIO PORTC
//			<o0.0..3> PORT0
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_HS_ULPI_STP)
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT1
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 (ETH_MDC)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT2
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 (SPI2_MISO)
//				<6=> AF6 (I2S2ext_SD)
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_HS_ULPI_DIR)
//				<11=> AF11 (ETH _MII_TXD2)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT3
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 (SPI2_MOSI , I2S2_SD)
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_HS_ULPI_NXT)
//				<11=> AF11 (ETH_MII_TX_CLK)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT4
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 (ETH_MII_RXD0 , ETH_RMII_RXD0)
//				<12=> AF12 
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT5
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 (ETH_MII_RXD1 , ETH_RMII_RXD1)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT6
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM3_CH1)
//				<3=> AF3 (TIM8_CH1)
//				<4=> AF4
//				<5=> AF5 (I2S2_MCK)
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 (USART6_TX)
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (SDIO_D6)
//				<13=> AF13 (DCMI_D0)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT7
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM3_CH2)
//				<3=> AF3 (TIM8_CH2)
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 (I2S3_MCK)
//				<7=> AF7 
//				<8=> AF8 (USART6_RX)
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (SDIO_D7)
//				<13=> AF13 (DCMI_D1)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)


#define GPIOC_AFRL_VALUE	0x00BBBBB0

//			<o0.0..3> PORT8
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM3_CH3)
//				<3=> AF3 (TIM8_CH3)
//				<4=> AF4 
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 (USART6_CK)
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (SDIO_D0)
//				<13=> AF13 (DCMI_D2)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT9
//				<0=> AF0 (MCO2)
//				<1=> AF1 
//				<2=> AF2 (TIM3_CH4)
//				<3=> AF3 (TIM8_CH4)
//				<4=> AF4 (I2C3_SDA)
//				<5=> AF5 (I2S_CKIN)
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (SDIO_D1)
//				<13=> AF13 (DCMI_D3)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT10
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6 (SPI3_SCK/I2S3_CK)
//				<7=> AF7 (USART3_TX)
//				<8=> AF8 (UART4_TX)
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (SDIO_D2)
//				<13=> AF13 (DCMI_D8)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT11
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 (I2S3ext_SD)
//				<6=> AF6 (SPI3_MISO)
//				<7=> AF7 (USART3_RX)
//				<8=> AF8 (UART4_RX)
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (SDIO_D3)
//				<13=> AF13 (DCMI_D4)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT12
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 (SPI3_MOSI , I2S3_SD)
//				<7=> AF7 (USART3_CK)
//				<8=> AF8 (UART5_TX)
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (SDIO_CK)
//				<13=> AF13 (DCMI_D9)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT13
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT14
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT15
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//		</h>

#define GPIOC_AFRH_VALUE	0x00000000


//		<h> GPIO PORTD
//			<o0.0..3> PORT0
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (CAN1_RX)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_D2)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT1
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (CAN1_TX)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_D3)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT2
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM3_ETR)
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 (UART5_RX)
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (SDIO_CMD)
//				<13=> AF13 (DCMI_D11)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT3
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART2_CTS)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_CLK)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT4
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 (USART2_RTS)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_NOE)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT5
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 (USART2_TX)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_NWE)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT6
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 (USART2_RX)
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_NWAIT)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT7
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 (USART2_CK)
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_NE1/FSMC_NCE2)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)


#define GPIOD_AFRL_VALUE	0x00000000

//			<o0.0..3> PORT8
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART3_TX)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_D13)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT9
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART3_RX)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_D14)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT10
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART3_CK)
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_D15)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT11
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 (USART3_CTS)
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_A16)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT12
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM4_CH1)
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 (USART3_RTS)
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_A17)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT13
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM4_CH2)
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_A18)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT14
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM4_CH3)
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_D0)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT15
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM4_CH4)
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_D1)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//		</h>

#define GPIOD_AFRH_VALUE	0x00000077


//		<h> GPIO PORTE
//			<o0.0..3> PORT0
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM4_ETR)
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_NBL0)
//				<13=> AF13 (DCMI_D2)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT1
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_NBL1)
//				<13=> AF13 (DCMI_D3)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT2
//				<0=> AF0 (TRACECLK)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 (ETH _MII_TXD3)
//				<12=> AF12 (FSMC_A23)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT3
//				<0=> AF0 (TRACED0)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_A19)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT4
//				<0=> AF0 (TRACED1)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A20)
//				<13=> AF13 (DCMI_D4)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT5
//				<0=> AF0 (TRACED2)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 (TIM9_CH1)
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A21)
//				<13=> AF13 (DCMI_D6)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT6
//				<0=> AF0 (TRACED3)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 (TIM9_CH2)
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A22)
//				<13=> AF13 (DCMI_D7)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT7
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_ETR)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_D4)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)


#define GPIOE_AFRL_VALUE	0x00000000

//			<o0.0..3> PORT8
//				<0=> AF0 
//				<1=> AF1 (TIM1_CH1N)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_D5)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT9
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH1)
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_D6)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT10
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH2N)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_D7)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT11
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH2)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_D8)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT12
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH3N)
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_D9)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT13
//				<0=> AF0 
//				<1=> AF1 (TIM1_CH3)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_D10)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT14
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_CH4)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 
//				<14=> AF14 (FSMC_D11)
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT15
//				<0=> AF0 (IO)
//				<1=> AF1 (TIM1_BKIN)
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_D12)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//		</h>

#define GPIOE_AFRH_VALUE	0x00000000



//		<h> GPIO PORTF
//			<o0.0..3> PORT0
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 (I2C2_SDA)
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A0)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT1
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 (I2C2_SCL)
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11
//				<12=> AF12 (FSMC_A1)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT2
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 (I2C2_SMBA)
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A2)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT3
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_A3)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT4
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A4)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT5
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_A5)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT6
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 (TIM10_CH1)
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_NIORD)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT7
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 (TIM11_CH1)
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_NREG)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)


#define GPIOF_AFRL_VALUE	0x00000000

//			<o0.0..3> PORT8
//				<0=> AF0 
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (TIM13_CH1)
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_NIOWR)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT9
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (TIM14_CH1)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_CD)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT10
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_INTR)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT11
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D12)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT12
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_A6)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT13
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_A7)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT14
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A8)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT15
//				<0=> AF0 
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A9)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//		</h>

#define GPIOF_AFRH_VALUE	0x00000000

//		<h> GPIO PORTG
//			<o0.0..3> PORT0
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A10)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT1
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A11)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT2
//				<0=> AF0 (IO)
//				<1=> AF1
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A12)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT3
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11
//				<12=> AF12 (FSMC_A13)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT4
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_A14)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT5
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_A15)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT6
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_INT2)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT7
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 (USART6_CK)
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_INT3)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)


#define GPIOG_AFRL_VALUE	0x00000000

//			<o0.0..3> PORT8
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 (USART6_RTS)
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 (ETH _PPS_OUT)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT9
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 (USART6_RX)
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12 (FSMC_NE2/FSMC_NCE3)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT10
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_NCE4_1/FSMC_NE3)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT11
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 (ETH_MII_TX_EN , ETH_RMII_TX_EN)
//				<12=> AF12 (FSMC_NCE4_2)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT12
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 (USART6_RTS)
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 (FSMC_NE4)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT13
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 (UART6_CTS)
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 (ETH_MII_TXD0 , ETH_RMII_TXD0)
//				<12=> AF12(FSMC_A24)
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT14
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 (USART6_TX)
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 (ETH_MII_TXD1 , ETH_RMII_TXD1)
//				<12=> AF12 (FSMC_A25)
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT15
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 (USART6_CTS)
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D13)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//		</h>

#define GPIOG_AFRH_VALUE	0x00000000

//		<h> GPIO PORTH
//			<o0.0..3> PORT0
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT1
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT2
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 (ETH _MII_CRS)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT3
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 (ETH _MII_COL)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT4
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 (I2C2_SCL)
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 (OTG_HS_ULPI_NXT)
//				<11=> AF11 
//				<12=> AF12 
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT5
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 (I2C2_SDA)
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT6
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 (I2C2_SMBA)
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (TIM12_CH1)
//				<10=> AF10
//				<11=> AF11 (ETH _MII_RXD2)
//				<12=> AF12
//				<13=> AF13 
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT7
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 (I2C3_SCL)
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 (ETH_MII_RXD3)
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)


#define GPIOH_AFRL_VALUE	0x00000000

//			<o0.0..3> PORT8
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4 (I2C3_SDA)
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_HSYNC)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT9
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4 (I2C3_SMBA)
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (TIM12_CH2)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D0)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT10
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM5_CH1)
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D1)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT11
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM5_CH2)
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D2)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT12
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 (TIM5_CH3)
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8
//				<9=> AF9 
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12 
//				<13=> AF13 (DCMI_D3)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT13
//				<0=> AF0 
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 (TIM8_CH1N)
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 (CAN1_TX)
//				<10=> AF10 
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT14
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 (TIM8_CH2N)
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D4)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT15
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 (TIM8_CH3N)
//				<4=> AF4
//				<5=> AF5 
//				<6=> AF6 
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9 
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D11)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//		</h>

#define GPIOH_AFRH_VALUE	0x00000000


//		<h> GPIO PORTI
//			<o0.0..3> PORT0
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2 (TIM5_CH4)
//				<3=> AF3  
//				<4=> AF4
//				<5=> AF5 (SPI2_NSS , I2S2_WS)
//				<6=> AF6
//				<7=> AF7  
//				<8=> AF8  
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11  
//				<12=> AF12
//				<13=> AF13 (DCMI_D13)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT1
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2  
//				<3=> AF3
//				<4=> AF4
//				<5=> AF5 (SPI2_SCK , I2S2_CK)
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8  
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11  
//				<12=> AF12
//				<13=> AF13 (DCMI_D8)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT2
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2  
//				<3=> AF3 (TIM8_CH4)
//				<4=> AF4
//				<5=> AF5 (SPI2_MISO)
//				<6=> AF6 (I2S2ext_SD)
//				<7=> AF7  
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11  
//				<12=> AF12
//				<13=> AF13 (DCMI_D9)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT3
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2  
//				<3=> AF3 (TIM8_ETR)
//				<4=> AF4
//				<5=> AF5 (SPI2_MOSI , I2S2_SD)
//				<6=> AF6
//				<7=> AF7  
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10  
//				<11=> AF11  
//				<12=> AF12
//				<13=> AF13 (DCMI_D10)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.16..19> PORT4
//				<0=> AF0 (IO)
//				<1=> AF1 
//				<2=> AF2 
//				<3=> AF3 (TIM8_BKIN)
//				<4=> AF4
//				<5=> AF5  
//				<6=> AF6  
//				<7=> AF7  
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12  
//				<13=> AF13 (DCMI_D5)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.20..23> PORT5
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2 
//				<3=> AF3 (TIM8_CH1)
//				<4=> AF4
//				<5=> AF5  
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10  
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_VSYNC)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.24..27> PORT6
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2  
//				<3=> AF3 (TIM8_CH2)
//				<4=> AF4
//				<5=> AF5  
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9  
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13 (DCMI_D6)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.28..31> PORT7
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2  
//				<3=> AF3 (TIM8_CH3)
//				<4=> AF4
//				<5=> AF5  
//				<6=> AF6
//				<7=> AF7 
//				<8=> AF8 
//				<9=> AF9  
//				<10=> AF10
//				<11=> AF11  
//				<12=> AF12
//				<13=> AF13 (DCMI_D7)
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)


#define GPIOI_AFRL_VALUE	0x00000000

//			<o0.0..3> PORT8
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4  
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7  
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10  
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.4..7> PORT9
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2 
//				<3=> AF3
//				<4=> AF4  
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7  
//				<8=> AF8 
//				<9=> AF9 (CAN1_RX)
//				<10=> AF10
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13  
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.8..11> PORT10
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7  
//				<8=> AF8 
//				<9=> AF9
//				<10=> AF10  
//				<11=> AF11 (ETH_MII_RX_ER)
//				<12=> AF12
//				<13=> AF13  
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//			<o0.12..15> PORT11
//				<0=> AF0 (IO)
//				<1=> AF1  
//				<2=> AF2 
//				<3=> AF3 
//				<4=> AF4
//				<5=> AF5
//				<6=> AF6
//				<7=> AF7  
//				<8=> AF8 
//				<9=> AF9  
//				<10=> AF10 (OTG_HS_ULPI_DIR)
//				<11=> AF11 
//				<12=> AF12
//				<13=> AF13
//				<14=> AF14
//				<15=> AF15 (EVENTOUT)
//		</h>

#define GPIOI_AFRH_VALUE		0x00000000

// 	</h> GPIO Alternate Function Low Register (GPIOx_AFRL)
//</h> GPIO Registers


//<h> System Configuration Controller (SYSCFG)
//	<h> SYSCFG Memory Remap Register (SYSCFG_MEMRMP)
//		<o0.0..1> Memory mapping selection (MEM_MODE)
//			<0=> Main Flash memory at 0x00000000
//			<1=> System Flash memory at 0x00000000
//			<2=> FSMC Bank1 (NOR/PSRAM 1 and 2) at 0x00000000
//			<3=> Embedded SRAM (SRAM1) at 0x00000000
//	</h>

#define SYSCFG_MEMRMP_VALUE		0x00000000

//	<h> SYSCFG Peripheral Mode Configuration Register (SYSCFG_PMC)
//		<o0.23> Ethernet PHY interface selection (MII_RMII_SEL)
//			<0=> MII interface
//			<1=> RMII PHY interface 
//	</h>

#define SYSCFG_PMC_VALUE		0x00000000


//	<h> SYSCFG External Interrupt Configuration Register 1/2/3/4 (SYSCFG_EXTICR1/2/3/4)
//		<o0.0..3> EXTI0 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.4..7> EXTI 1 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.8..11> EXTI 2 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.12..15> EXTI 3 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI

#define SYSCFG_EXTICR1_VALUE 	0x00000000

//		<o0.0..3> EXTI 4 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.4..7> EXTI 5 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.8..11> EXTI 6 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.12..15> EXTI 7 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI

#define SYSCFG_EXTICR2_VALUE 	0x00000000


//		<o0.0..3> EXTI 8 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.4..7> EXTI 9 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.8..11> EXTI 10 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.12..15> EXTI 11 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI

#define SYSCFG_EXTICR3_VALUE 	0x00000000


//		<o0.0..3> EXTI 12 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.4..7> EXTI 13 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.8..11> EXTI 14 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//		<o0.12..15> EXTI 15 Input Source
//			<0=> PORTA
//			<1=> PORTB
//			<2=> PORTC
//			<3=> PORTD
//			<4=> PORTE
//			<5=> PORTF
//			<6=> PORTG
//			<7=> PORTH
//			<8=> PORTI
//	</h>

#define SYSCFG_EXTICR4_VALUE 	0x00000000

//	<h> Compensation Cell Control Register (SYSCFG_CMPCR)
//	<q0.0> Compensation cell power-down Enable (CMP_PD)
//	<i> 0: I/O compensation cell power-down mode
//	<i> 1: I/O compensation cell enabled
//	</h>

#define SYSCFG_CMPCR_VALUE 		0x00000000
	
//</h>

//<h>Universal Synchronous Asynchronous Receiver Transmitter(USART)
//<e1.13> USART1
//	<h> Baud Rate Register (USART_BRR)
//		<o0.4..15> mantissa of USARTDIV (DIV_Mantissa[11:0]) <0-4095>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//		<o0.0..3> fraction of USARTDIV (DIV_Fraction[3:0]) <0-15>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//	</h>

#define USART1_BRR_VALUE		0x0000028B

//	<h> Control register 1 (USART_CR1)
//		<o0.15> Oversampling mode (OVER8)
//			<0=> Oversampling by 16
//			<1=> Oversampling by 8
//		<o0.12> Word length (M)
//			<0=> 1 Start bit, 8 Data bits, n Stop bit
//			<1=> 1 Start bit, 9 Data bits, n Stop bit
//		<o0.11> Wakeup method (WAKE)
//			<0=> Idle Line
//			<1=> Address Mark
//		<q0.10> Parity control enable (PCE)
//		<o0.9> Parity selection (PS)
//			<0=> Even parity
//			<1=> Odd parity
//		<q0.8> PE interrupt enable (PEIE)
//		<q0.7> TXE interrupt enable (TXEIE)
//		<q0.6> Transmission complete interrupt enable (TCIE)
//		<q0.5> RXNE interrupt enable (RXNEIE)
//		<q0.4> IDLE interrupt enable (IDLEIE)
//		<q0.3> Transmitter enable (TE)
//		<q0.2> Receiver enable (RE)
//		<o0.1> Receiver wakeup (RWU)
//			<0=> Active Mode
//			<1=> Mute Mode
//	</h>

#define USART1_CR1_VALUE		0x0000200C

//	<h> Control Register 2 (USART_CR2)
//		<q0.14> LIN mode enable (LINEN)
//		<o0.12..13> STOP bits (STOP)
//			<0=> 1 Stop bit
//			<1=> 0.5 Stop bit
//			<2=> 2 Stop bits
//			<3=> 1.5 Stop bit
//		<q0.11> Clock enable (CLKEN)
//		<o0.10> Clock polarity (CPOL)
//			<0=> Steady low value on SCLK pin
//			<1=> Steady high value on SCLK pin
//		<o0.9> Clock phase (CPHA)
//			<0=> The 1st clock trans is the 1st data capture edge
//			<1=> The 2nd clock trans is the 1st data capture edge
//		<o0.8> Last bit clock pulse (LBCL)
//			<0=> The last clock pulse not output to SCLK pin
//			<1=> The last clock pulse output to SCLK pin
//		<q0.6> LIN break detection interrupt enable (LBDIE)
//		<o0.5> lin break detection length (LBDL)
//			<0=> 10-bit break detection
//			<0=> 11-bit break detection
//		<o0.0..3> Address of the USART node	(ADD[3:0])	<0-7>
//	</h>

#define USART1_CR2_VALUE		0x00000000

//	<h> Control register 3 (USART_CR3)
//		<o0.1> sample bit method (ONEBIT)
//			<0=> Three sample bit method
//			<1=> One sample bit method
//		<q0.10> CTS interrupt enable (CTSIE)
//		<q0.9> CTS enable (CTSE)
//		<q0.8> RTS enable (RTSE)
//		<q0.7> DMA enable transmitter (DMAT)
//		<q0.6> DMA enable receiver (DMAR)
//		<q0.5> Smartcard mode enable (SCEN)
//		<q0.4> Smartcard NACK enable (NACK)
//		<q0.3> Half-duplex selection (HDSEL)
//		<o0.2> IrDA low-power (IRLP)
//			<0=> Normal mode
//			<1=> Low-power mode
//		<q0.1> IrDA mode enable (IREN)
//		<q0.0> Error interrupt enable (EIE)
//	</h>

#define USART1_CR3_VALUE		0x000000C0

//</e>


//<e1.13> USART2
//	<h> Baud Rate Register (USART_BRR)
//		<o0.4..15> mantissa of USARTDIV (DIV_Mantissa[11:0]) <0-4095>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//		<o0.0..3> fraction of USARTDIV (DIV_Fraction[3:0]) <0-15>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//	</h>

#define USART2_BRR_VALUE		0x00000000

//	<h> Control register 1 (USART_CR1)
//		<o0.15> Oversampling mode (OVER8)
//			<0=> Oversampling by 16
//			<1=> Oversampling by 8
//		<o0.12> Word length (M)
//			<0=> 1 Start bit, 8 Data bits, n Stop bit
//			<1=> 1 Start bit, 9 Data bits, n Stop bit
//		<o0.11> Wakeup method (WAKE)
//			<0=> Idle Line
//			<1=> Address Mark
//		<q0.10> Parity control enable (PCE)
//		<o0.9> Parity selection (PS)
//			<0=> Even parity
//			<1=> Odd parity
//		<q0.8> PE interrupt enable (PEIE)
//		<q0.7> TXE interrupt enable (TXEIE)
//		<q0.6> Transmission complete interrupt enable (TCIE)
//		<q0.5> RXNE interrupt enable (RXNEIE)
//		<q0.4> IDLE interrupt enable (IDLEIE)
//		<q0.3> Transmitter enable (TE)
//		<q0.2> Receiver enable (RE)
//		<o0.1> Receiver wakeup (RWU)
//			<0=> Active Mode
//			<1=> Mute Mode
//	</h>

#define USART2_CR1_VALUE		0x00000000

//	<h> Control Register 2 (USART_CR2)
//		<q0.14> LIN mode enable (LINEN)
//		<o0.12..13> STOP bits (STOP)
//			<0=> 1 Stop bit
//			<1=> 0.5 Stop bit
//			<2=> 2 Stop bits
//			<3=> 1.5 Stop bit
//		<q0.11> Clock enable (CLKEN)
//		<o0.10> Clock polarity (CPOL)
//			<0=> Steady low value on SCLK pin
//			<1=> Steady high value on SCLK pin
//		<o0.9> Clock phase (CPHA)
//			<0=> The 1st clock trans is the 1st data capture edge
//			<1=> The 2nd clock trans is the 1st data capture edge
//		<o0.8> Last bit clock pulse (LBCL)
//			<0=> The last clock pulse not output to SCLK pin
//			<1=> The last clock pulse output to SCLK pin
//		<q0.6> LIN break detection interrupt enable (LBDIE)
//		<o0.5> lin break detection length (LBDL)
//			<0=> 10-bit break detection
//			<0=> 11-bit break detection
//		<o0.0..3> Address of the USART node	(ADD[3:0])	<0-7>
//	</h>

#define USART2_CR2_VALUE		0x00000000

//	<h> Control register 3 (USART_CR3)
//		<o0.1> sample bit method (ONEBIT)
//			<0=> Three sample bit method
//			<1=> One sample bit method
//		<q0.10> CTS interrupt enable (CTSIE)
//		<q0.9> CTS enable (CTSE)
//		<q0.8> RTS enable (RTSE)
//		<q0.7> DMA enable transmitter (DMAT)
//		<q0.6> DMA enable receiver (DMAR)
//		<q0.5> Smartcard mode enable (SCEN)
//		<q0.4> Smartcard NACK enable (NACK)
//		<q0.3> Half-duplex selection (HDSEL)
//		<o0.2> IrDA low-power (IRLP)
//			<0=> Normal mode
//			<1=> Low-power mode
//		<q0.1> IrDA mode enable (IREN)
//		<q0.0> Error interrupt enable (EIE)
//	</h>

#define USART2_CR3_VALUE		0x00000000

//</e>


//<e1.13> USART3
//	<h> Baud Rate Register (USART_BRR)
//		<o0.4..15> mantissa of USARTDIV (DIV_Mantissa[11:0]) <0-4095>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//		<o0.0..3> fraction of USARTDIV (DIV_Fraction[3:0]) <0-15>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//	</h>

#define USART3_BRR_VALUE		0x00007A12

//	<h> Control register 1 (USART_CR1)
//		<o0.15> Oversampling mode (OVER8)
//			<0=> Oversampling by 16
//			<1=> Oversampling by 8
//		<o0.12> Word length (M)
//			<0=> 1 Start bit, 8 Data bits, n Stop bit
//			<1=> 1 Start bit, 9 Data bits, n Stop bit
//		<o0.11> Wakeup method (WAKE)
//			<0=> Idle Line
//			<1=> Address Mark
//		<q0.10> Parity control enable (PCE)
//		<o0.9> Parity selection (PS)
//			<0=> Even parity
//			<1=> Odd parity
//		<q0.8> PE interrupt enable (PEIE)
//		<q0.7> TXE interrupt enable (TXEIE)
//		<q0.6> Transmission complete interrupt enable (TCIE)
//		<q0.5> RXNE interrupt enable (RXNEIE)
//		<q0.4> IDLE interrupt enable (IDLEIE)
//		<q0.3> Transmitter enable (TE)
//		<q0.2> Receiver enable (RE)
//		<o0.1> Receiver wakeup (RWU)
//			<0=> Active Mode
//			<1=> Mute Mode
//	</h>

#define USART3_CR1_VALUE		0x0000200C

//	<h> Control Register 2 (USART_CR2)
//		<q0.14> LIN mode enable (LINEN)
//		<o0.12..13> STOP bits (STOP)
//			<0=> 1 Stop bit
//			<1=> 0.5 Stop bit
//			<2=> 2 Stop bits
//			<3=> 1.5 Stop bit
//		<q0.11> Clock enable (CLKEN)
//		<o0.10> Clock polarity (CPOL)
//			<0=> Steady low value on SCLK pin
//			<1=> Steady high value on SCLK pin
//		<o0.9> Clock phase (CPHA)
//			<0=> The 1st clock trans is the 1st data capture edge
//			<1=> The 2nd clock trans is the 1st data capture edge
//		<o0.8> Last bit clock pulse (LBCL)
//			<0=> The last clock pulse not output to SCLK pin
//			<1=> The last clock pulse output to SCLK pin
//		<q0.6> LIN break detection interrupt enable (LBDIE)
//		<o0.5> lin break detection length (LBDL)
//			<0=> 10-bit break detection
//			<0=> 11-bit break detection
//		<o0.0..3> Address of the USART node	(ADD[3:0])	<0-7>
//	</h>

#define USART3_CR2_VALUE		0x00000000

//	<h> Control register 3 (USART_CR3)
//		<o0.1> sample bit method (ONEBIT)
//			<0=> Three sample bit method
//			<1=> One sample bit method
//		<q0.10> CTS interrupt enable (CTSIE)
//		<q0.9> CTS enable (CTSE)
//		<q0.8> RTS enable (RTSE)
//		<q0.7> DMA enable transmitter (DMAT)
//		<q0.6> DMA enable receiver (DMAR)
//		<q0.5> Smartcard mode enable (SCEN)
//		<q0.4> Smartcard NACK enable (NACK)
//		<q0.3> Half-duplex selection (HDSEL)
//		<o0.2> IrDA low-power (IRLP)
//			<0=> Normal mode
//			<1=> Low-power mode
//		<q0.1> IrDA mode enable (IREN)
//		<q0.0> Error interrupt enable (EIE)
//	</h>

#define USART3_CR3_VALUE		0x00000000

//</e>


//<e1.13> UART4
//	<h> Baud Rate Register (USART_BRR)
//		<o0.4..15> mantissa of USARTDIV (DIV_Mantissa[11:0]) <0-4095>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//		<o0.0..3> fraction of USARTDIV (DIV_Fraction[3:0]) <0-15>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//	</h>

#define USART4_BRR_VALUE		0x00000000

//	<h> Control register 1 (USART_CR1)
//		<o0.15> Oversampling mode (OVER8)
//			<0=> Oversampling by 16
//			<1=> Oversampling by 8
//		<o0.12> Word length (M)
//			<0=> 1 Start bit, 8 Data bits, n Stop bit
//			<1=> 1 Start bit, 9 Data bits, n Stop bit
//		<o0.11> Wakeup method (WAKE)
//			<0=> Idle Line
//			<1=> Address Mark
//		<q0.10> Parity control enable (PCE)
//		<o0.9> Parity selection (PS)
//			<0=> Even parity
//			<1=> Odd parity
//		<q0.8> PE interrupt enable (PEIE)
//		<q0.7> TXE interrupt enable (TXEIE)
//		<q0.6> Transmission complete interrupt enable (TCIE)
//		<q0.5> RXNE interrupt enable (RXNEIE)
//		<q0.4> IDLE interrupt enable (IDLEIE)
//		<q0.3> Transmitter enable (TE)
//		<q0.2> Receiver enable (RE)
//		<o0.1> Receiver wakeup (RWU)
//			<0=> Active Mode
//			<1=> Mute Mode
//	</h>

#define USART4_CR1_VALUE		0x00000000

//	<h> Control Register 2 (USART_CR2)
//		<q0.14> LIN mode enable (LINEN)
//		<o0.12..13> STOP bits (STOP)
//			<0=> 1 Stop bit
//			<1=> 0.5 Stop bit
//			<2=> 2 Stop bits
//			<3=> 1.5 Stop bit
//		<q0.6> LIN break detection interrupt enable (LBDIE)
//		<o0.5> lin break detection length (LBDL)
//			<0=> 10-bit break detection
//			<0=> 11-bit break detection
//		<o0.0..3> Address of the USART node	(ADD[3:0])	<0-7>
//	</h>

#define USART4_CR2_VALUE		0x00000000

//	<h> Control register 3 (USART_CR3)
//		<o0.1> sample bit method (ONEBIT)
//			<0=> Three sample bit method
//			<1=> One sample bit method
//		<q0.7> DMA enable transmitter (DMAT)
//		<q0.6> DMA enable receiver (DMAR)
//		<q0.3> Half-duplex selection (HDSEL)
//		<o0.2> IrDA low-power (IRLP)
//			<0=> Normal mode
//			<1=> Low-power mode
//		<q0.1> IrDA mode enable (IREN)
//		<q0.0> Error interrupt enable (EIE)
//	</h>

#define USART4_CR3_VALUE		0x00000000

//</e>


//<e1.13> UART5
//	<h> Baud Rate Register (USART_BRR)
//		<o0.4..15> mantissa of USARTDIV (DIV_Mantissa[11:0]) <0-4095>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//		<o0.0..3> fraction of USARTDIV (DIV_Fraction[3:0]) <0-15>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//	</h>

#define USART5_BRR_VALUE		0x00000000

//	<h> Control register 1 (USART_CR1)
//		<o0.15> Oversampling mode (OVER8)
//			<0=> Oversampling by 16
//			<1=> Oversampling by 8
//		<o0.12> Word length (M)
//			<0=> 1 Start bit, 8 Data bits, n Stop bit
//			<1=> 1 Start bit, 9 Data bits, n Stop bit
//		<o0.11> Wakeup method (WAKE)
//			<0=> Idle Line
//			<1=> Address Mark
//		<q0.10> Parity control enable (PCE)
//		<o0.9> Parity selection (PS)
//			<0=> Even parity
//			<1=> Odd parity
//		<q0.8> PE interrupt enable (PEIE)
//		<q0.7> TXE interrupt enable (TXEIE)
//		<q0.6> Transmission complete interrupt enable (TCIE)
//		<q0.5> RXNE interrupt enable (RXNEIE)
//		<q0.4> IDLE interrupt enable (IDLEIE)
//		<q0.3> Transmitter enable (TE)
//		<q0.2> Receiver enable (RE)
//		<o0.1> Receiver wakeup (RWU)
//			<0=> Active Mode
//			<1=> Mute Mode
//	</h>

#define USART5_CR1_VALUE		0x00000000

//	<h> Control Register 2 (USART_CR2)
//		<q0.14> LIN mode enable (LINEN)
//		<o0.12..13> STOP bits (STOP)
//			<0=> 1 Stop bit
//			<1=> 0.5 Stop bit
//			<2=> 2 Stop bits
//			<3=> 1.5 Stop bit
//		<q0.6> LIN break detection interrupt enable (LBDIE)
//		<o0.5> lin break detection length (LBDL)
//			<0=> 10-bit break detection
//			<0=> 11-bit break detection
//		<o0.0..3> Address of the USART node	(ADD[3:0])	<0-7>
//	</h>

#define USART5_CR2_VALUE		0x00000000

//	<h> Control register 3 (USART_CR3)
//		<o0.1> sample bit method (ONEBIT)
//			<0=> Three sample bit method
//			<1=> One sample bit method
//		<q0.7> DMA enable transmitter (DMAT)
//		<q0.6> DMA enable receiver (DMAR)
//		<q0.3> Half-duplex selection (HDSEL)
//		<o0.2> IrDA low-power (IRLP)
//			<0=> Normal mode
//			<1=> Low-power mode
//		<q0.1> IrDA mode enable (IREN)
//		<q0.0> Error interrupt enable (EIE)
//	</h>

#define USART5_CR3_VALUE		0x00000000

//</e>


//<e1.13> USART6
//	<h> Baud Rate Register (USART_BRR)
//		<o0.4..15> mantissa of USARTDIV (DIV_Mantissa[11:0]) <0-4095>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//		<o0.0..3> fraction of USARTDIV (DIV_Fraction[3:0]) <0-15>
//		<i> Baud rate for standard USART
//		<i>-                                          fCK
//		<i>Tx/Rx baud = -------------------------------
//		<i>-                           8×(2–OVER8)×USARTDIV
//	</h>

#define USART6_BRR_VALUE		0x00000000

//	<h> Control register 1 (USART_CR1)
//		<o0.15> Oversampling mode (OVER8)
//			<0=> Oversampling by 16
//			<1=> Oversampling by 8
//		<o0.12> Word length (M)
//			<0=> 1 Start bit, 8 Data bits, n Stop bit
//			<1=> 1 Start bit, 9 Data bits, n Stop bit
//		<o0.11> Wakeup method (WAKE)
//			<0=> Idle Line
//			<1=> Address Mark
//		<q0.10> Parity control enable (PCE)
//		<o0.9> Parity selection (PS)
//			<0=> Even parity
//			<1=> Odd parity
//		<q0.8> PE interrupt enable (PEIE)
//		<q0.7> TXE interrupt enable (TXEIE)
//		<q0.6> Transmission complete interrupt enable (TCIE)
//		<q0.5> RXNE interrupt enable (RXNEIE)
//		<q0.4> IDLE interrupt enable (IDLEIE)
//		<q0.3> Transmitter enable (TE)
//		<q0.2> Receiver enable (RE)
//		<o0.1> Receiver wakeup (RWU)
//			<0=> Active Mode
//			<1=> Mute Mode
//	</h>

#define USART6_CR1_VALUE		0x00000000

//	<h> Control Register 2 (USART_CR2)
//		<q0.14> LIN mode enable (LINEN)
//		<o0.12..13> STOP bits (STOP)
//			<0=> 1 Stop bit
//			<1=> 0.5 Stop bit
//			<2=> 2 Stop bits
//			<3=> 1.5 Stop bit
//		<q0.11> Clock enable (CLKEN)
//		<o0.10> Clock polarity (CPOL)
//			<0=> Steady low value on SCLK pin
//			<1=> Steady high value on SCLK pin
//		<o0.9> Clock phase (CPHA)
//			<0=> The 1st clock trans is the 1st data capture edge
//			<1=> The 2nd clock trans is the 1st data capture edge
//		<o0.8> Last bit clock pulse (LBCL)
//			<0=> The last clock pulse not output to SCLK pin
//			<1=> The last clock pulse output to SCLK pin
//		<q0.6> LIN break detection interrupt enable (LBDIE)
//		<o0.5> lin break detection length (LBDL)
//			<0=> 10-bit break detection
//			<0=> 11-bit break detection
//		<o0.0..3> Address of the USART node	(ADD[3:0])	<0-7>
//	</h>

#define USART6_CR2_VALUE		0x00000000

//	<h> Control register 3 (USART_CR3)
//		<o0.1> sample bit method (ONEBIT)
//			<0=> Three sample bit method
//			<1=> One sample bit method
//		<q0.10> CTS interrupt enable (CTSIE)
//		<q0.9> CTS enable (CTSE)
//		<q0.8> RTS enable (RTSE)
//		<q0.7> DMA enable transmitter (DMAT)
//		<q0.6> DMA enable receiver (DMAR)
//		<q0.5> Smartcard mode enable (SCEN)
//		<q0.4> Smartcard NACK enable (NACK)
//		<q0.3> Half-duplex selection (HDSEL)
//		<o0.2> IrDA low-power (IRLP)
//			<0=> Normal mode
//			<1=> Low-power mode
//		<q0.1> IrDA mode enable (IREN)
//		<q0.0> Error interrupt enable (EIE)
//	</h>

#define USART6_CR3_VALUE		0x00000000

//</e>
//</h>


//<h> Direct Memory Access Registers (DMA)
//<h>Direct Memory Access 1 (DMA1)
//	<e0.0> Strem 0
//		<h> DMA stream 0 configuration register (DMA_S0CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA1_S0CR_VALUE		0x00000000

//		<h> DMA stream 0 FIFO control register (DMA_S0FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA1_S0FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 1
//		<h> DMA stream 1 configuration register (DMA_S1CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA1_S1CR_VALUE		0x00000000

//		<h> DMA stream 1 FIFO control register (DMA_S1FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA1_S1FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 2
//		<h> DMA stream 2 configuration register (DMA_S2CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA1_S2CR_VALUE		0x00000000

//		<h> DMA stream 2 FIFO control register (DMA_S2FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA1_S2FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 3
//		<h> DMA stream 3 configuration register (DMA_S3CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA1_S3CR_VALUE		0x00000000

//		<h> DMA stream 3 FIFO control register (DMA_S3FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA1_S3FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 4
//		<h> DMA stream 4 configuration register (DMA_S4CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA1_S4CR_VALUE		0x00000000

//		<h> DMA stream 4 FIFO control register (DMA_S4FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA1_S4FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 5
//		<h> DMA stream 5 configuration register (DMA_S5CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA1_S5CR_VALUE		0x00000000

//		<h> DMA stream 5 FIFO control register (DMA_S5FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA1_S5FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 6
//		<h> DMA stream 6 configuration register (DMA_S6CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA1_S6CR_VALUE		0x00000000

//		<h> DMA stream 6 FIFO control register (DMA_S6FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA1_S6FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 7
//		<h> DMA stream 7 configuration register (DMA_S7CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA1_S7CR_VALUE		0x00000000

//		<h> DMA stream 7 FIFO control register (DMA_S7FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA1_S7FCR_VALUE		0x00000000

//	</e> 
//	</h>


//<h>Direct Memory Access 2 (DMA2)
//	<e0.0> Strem 0
//		<h> DMA stream 0 configuration register (DMA_S0CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA2_S0CR_VALUE		0x00000000

//		<h> DMA stream 0 FIFO control register (DMA_S0FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA2_S0FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 1
//		<h> DMA stream 1 configuration register (DMA_S1CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA2_S1CR_VALUE		0x00000000

//		<h> DMA stream 1 FIFO control register (DMA_S1FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA2_S1FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 2
//		<h> DMA stream 2 configuration register (DMA_S2CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA2_S2CR_VALUE		0x00000000

//		<h> DMA stream 2 FIFO control register (DMA_S2FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA2_S2FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 3
//		<h> DMA stream 3 configuration register (DMA_S3CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA2_S3CR_VALUE		0x00000000

//		<h> DMA stream 3 FIFO control register (DMA_S3FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA2_S3FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 4
//		<h> DMA stream 4 configuration register (DMA_S4CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA2_S4CR_VALUE		0x08000000

//		<h> DMA stream 4 FIFO control register (DMA_S4FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA2_S4FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 5
//		<h> DMA stream 5 configuration register (DMA_S5CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA2_S5CR_VALUE		0x00000000

//		<h> DMA stream 5 FIFO control register (DMA_S5FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA2_S5FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 6
//		<h> DMA stream 6 configuration register (DMA_S6CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA2_S6CR_VALUE		0x00000000

//		<h> DMA stream 6 FIFO control register (DMA_S6FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA2_S6FCR_VALUE		0x00000000

//	</e> 


//	<e0.0> Strem 7
//		<h> DMA stream 7 configuration register (DMA_S7CR) 
//			<o0.25..27> Channel selection (CHSEL[2:0])
//				<0=> Channel 0
//				<1=> Channel 1
//				<2=> Channel 2
//				<3=> Channel 3
//				<4=> Channel 4
//				<5=> Channel 5
//				<6=> Channel 6
//				<7=> Channel 7
//			<o0.23..24> Memory burst transfer configuration (MBURST)
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.21..22> Peripheral burst transfer configuration (PBURST[1:0])
//				<0=> single transfer
//				<1=> INCR4 
//				<2=> INCR8 
//				<3=> INCR16
//			<o0.19> Current target (only in double buffer mode) (CT)
//				<0=> The current is Memory 0 (DMA_SxM0AR pointer)
//				<1=> The current is Memory 1 (DMA_SxM1AR pointer)
//			<o0.18> Double buffer mode (DBM)
//			<o0.16..17> Priority level (PL[1:0])
//				<0=> Low
//				<1=> Medium
//				<2=> High
//				<2=> Very high
//			<o0.15> Peripheral increment offset size (PINCOS)
//				<0=> increment by PSIZE
//				<1=> increment by 4 	
//			<o0.13..14> Memory data size (MSIZE[1:0])
//				<0=> byte (8-bit)
//				<1=> half-word (16-bit)
//				<2=> word (32-bit)
//			<o0.11..12> Peripheral data size (PSIZE[1:0])
//				<0=> Byte (8-bit)
//				<1=> Half-word (16-bit)
//				<2=> Word (32-bit)
//			<o0.10> Memory increment mode (MINC)
//				<0=> fixed address 
//				<1=> increment by MSIZE
//			<o0.9> Peripheral increment mode (PINC)
//				<0=> fixed address
//				<1=> increment by PSIZE
//			<q0.8> Circular mode enable(CIRC)
//			<o0.6..7> Data transfer direction (DIR[1:0])
//				<0=> Peripheral-to-memory
//				<1=> Memory-to-peripheral
//				<2=> Memory-to-memory
//			<o0.5> Peripheral flow controller (PFCTRL)
//				<0=> DMA
//				<1=> Peripheral 
//			<q0.4> Transfer complete interrupt enable (TCIE)
//			<q0.3> Half transfer interrupt enable (HTIE)
//			<q0.2> Transfer error interrupt enable (TEIE)
//			<q0.1> Direct mode error interrupt enable (DMEIE)			
//		</h>

#define DMA2_S7CR_VALUE		0x08000441

//		<h> DMA stream 7 FIFO control register (DMA_S7FCR)
//			<q0.7> FIFO error interrupt enable (FEIE)
//			<o0.2> DMDIS: Direct mode disable
//				<0=> Direct mode enabled
//				<1=> Direct mode disabled
//			<o0.0..1> FIFO threshold selection (FTH[1:0])
//				<0=> 1/4 full FIFO
//				<1=> 1/2 full FIFO
//				<2=> 3/4 full FIFO
//				<3=> full FIFO
//		</h>

#define DMA2_S7FCR_VALUE	0x00000000

//	</e> 
//	</h>
//</h>

//<h> Serial Peripheral Interface (SPI)
// <h> SPI1
//	<o2.11> I2S mode selection (I2SMOD)
//		<0=> SPI mode
//		<1=> I2S mode
//	<e0.6> SPI Mode
//		<h> SPI control register 1 (SPI_CR1)
//			<o0.15> Bidirectional data mode enable (BIDIMODE)
//				<0=> 2-line unidirectional
//				<1=> 1-line bidirectional
//			<o0.14> Output enable in bidirectional mode (BIDIOE)
//				<0=> Output disabled (receive-only mode)
//				<1=> Output enabled (transmit-only mode)
//			<o0.13> Hardware CRC calculation enable (CRCEN)
//			<o0.12> CRC transfer next (CRCNEXT)
//				<0=> Data phase (no CRC phase)
//				<1=> Next transfer is CRC (CRC phase)
//			<o0.11> Data frame format(DFF)
//				<0=> 8-bit 
//				<1=> 16-bit
//			<o0.10> Receive only (RXONLY)
//				<0=> Full duplex (Transmit and receive)
//				<1=> Output disabled (Receive-only mode)
//			<o0.9> Software slave management (SSM)
//				<0=> Software slave management disabled
//				<1=> Software slave management enabled
//			<o0.7>Frame format (LSBFIRST)
//				<0=> MSB transmitted first
//				<1=> LSB transmitted first
//			<o0.3..5> Baud rate control (BR[2:0])
//				<0=>Fpclk/2
//				<1=>Fpclk/4
//				<2=>Fpclk/8
//				<3=>Fpclk/16
//				<4=>Fpclk/32 
//				<5=>Fpclk/64
//				<6=>Fpclk/128
//				<7=>Fpclk/256
//			<o0.2> Master selection (MSTR)
//				<0=> Slave
//				<1=> Master
//			<o0.1> Clock polarity (CPOL)
//				<0=> CK to 0 when idle
//				<1=> CK to 1 when idle
//			<o0.0> Clock phase (CPHA)
//				<0=> First edge Capture 
//				<1=> Second edge Capture
//		</h>


#define SPI1_CR1_VALUE		0x0000085A

//		<h> SPI control register 2 (SPI_CR2)
//			<o0.7> Tx buffer empty interrupt enable (TXEIE)
//			<o0.6> RX buffer not empty interrupt enable (RXNEIE)
//			<o0.5> Error interrupt enable (ERRIE)
//			<o0.4> Frame format (FRF)
//				<0=> SPI Motorola mode
//				<1=> SPI TI mode
//			<o0.2> SS output enable (SSOE)
//			<o0.1> Tx buffer DMA enable (TXDMAEN)
//			<o0.0> Rx buffer DMA enable (RXDMAEN)
//		</h>
//	</e>
#define SPI1_CR2_VALUE		0x00000000

//	<e0.10> I2S Mode
//		<h> SPI_I2S configuration register (SPI_I2SCFGR)
//			<o0.8..9> I2S configuration mode (I2SCFG)
//				<0=> Slave - transmit
//				<1=> Slave - receive
//				<2=> Master - transmit
//				<3=> Master - receive
//			<o0.7> PCM frame synchronization (PCMSYNC)
//				<0=> Short frame synch
//				<1=> Long frame synch
//			<o0.4..5> I2S standard selection (I2SSTD)
//				<0=> I2S Philips standard.
//				<1=> MSB justified standard (left justified)
//				<2=> LSB justified standard (right justified)
//				<3=> PCM standard
//			<o0.3> Steady state clock polarity (CKPOL)
//				<0=> CK to 0 when idle
//				<1=> CK to 1 when idle
//			<o0.1..2> Data length to be transferred (DATLEN)
//				<0=> 16-bit
//				<1=> 24-bit
//				<2=> 32-bit
//			<o0.0> Channel length (number of bits per audio channel) (CHLEN) 
//				<0=> 16-bit wide
//				<1=> 32-bit wide
//		</h>

#define SPI1_I2SCFGR_VALUE		0x00000000

//		<h> SPI_I2S prescaler register (SPI_I2SPR)
//			<o0.9> Master clock output enable (MCKOE)
//			<o0.8> Odd factor for the prescaler (ODD)
//				<0=> real divider value is = I2SDIV *2
//				<1=> real divider value is = (I2SDIV * 2)+1
//			<o0.0..7> I2S Linear prescaler (I2SDIV)  <2-127> 
//		</h>
//	</e>

#define SPI1_I2SPR_VALUE		0x00000002

// </h> SPI1

// <h> SPI2
//	<o2.11> I2S mode selection (I2SMOD)
//		<0=> SPI mode
//		<1=> I2S mode
//	<e0.6> SPI Mode
//		<h> SPI control register 1 (SPI_CR1)
//			<o0.15> Bidirectional data mode enable (BIDIMODE)
//				<0=> 2-line unidirectional
//				<1=> 1-line bidirectional
//			<o0.14> Output enable in bidirectional mode (BIDIOE)
//				<0=> Output disabled (receive-only mode)
//				<1=> Output enabled (transmit-only mode)
//			<o0.13> Hardware CRC calculation enable (CRCEN)
//			<o0.12> CRC transfer next (CRCNEXT)
//				<0=> Data phase (no CRC phase)
//				<1=> Next transfer is CRC (CRC phase)
//			<o0.11> Data frame format(DFF)
//				<0=> 8-bit 
//				<1=> 16-bit
//			<o0.10> Receive only (RXONLY)
//				<0=> Full duplex (Transmit and receive)
//				<1=> Output disabled (Receive-only mode)
//			<o0.9> Software slave management (SSM)
//				<0=> Software slave management disabled
//				<1=> Software slave management enabled
//			<o0.7>Frame format (LSBFIRST)
//				<0=> MSB transmitted first
//				<1=> LSB transmitted first
//			<o0.3..5> Baud rate control (BR[2:0])
//				<0=>Fpclk/2
//				<1=>Fpclk/4
//				<2=>Fpclk/8
//				<3=>Fpclk/16
//				<4=>Fpclk/32 
//				<5=>Fpclk/64
//				<6=>Fpclk/128
//				<7=>Fpclk/256
//			<o0.2> Master selection (MSTR)
//				<0=> Slave
//				<1=> Master
//			<o0.1> Clock polarity (CPOL)
//				<0=> CK to 0 when idle
//				<1=> CK to 1 when idle
//			<o0.0> Clock phase (CPHA)
//				<0=> First edge Capture 
//				<1=> Second edge Capture
//		</h>


#define SPI2_CR1_VALUE		0x00000000

//		<h> SPI control register 2 (SPI_CR2)
//			<o0.7> Tx buffer empty interrupt enable (TXEIE)
//			<o0.6> RX buffer not empty interrupt enable (RXNEIE)
//			<o0.5> Error interrupt enable (ERRIE)
//			<o0.4> Frame format (FRF)
//				<0=> SPI Motorola mode
//				<1=> SPI TI mode
//			<o0.2> SS output enable (SSOE)
//			<o0.1> Tx buffer DMA enable (TXDMAEN)
//			<o0.0> Rx buffer DMA enable (RXDMAEN)
//		</h>
//	</e>
#define SPI2_CR2_VALUE		0x00000000

//	<e0.10> I2S Mode
//		<h> SPI_I2S configuration register (SPI_I2SCFGR)
//			<o0.8..9> I2S configuration mode (I2SCFG)
//				<0=> Slave - transmit
//				<1=> Slave - receive
//				<2=> Master - transmit
//				<3=> Master - receive
//			<o0.7> PCM frame synchronization (PCMSYNC)
//				<0=> Short frame synch
//				<1=> Long frame synch
//			<o0.4..5> I2S standard selection (I2SSTD)
//				<0=> I2S Philips standard.
//				<1=> MSB justified standard (left justified)
//				<2=> LSB justified standard (right justified)
//				<3=> PCM standard
//			<o0.3> Steady state clock polarity (CKPOL)
//				<0=> CK to 0 when idle
//				<1=> CK to 1 when idle
//			<o0.1..2> Data length to be transferred (DATLEN)
//				<0=> 16-bit
//				<1=> 24-bit
//				<2=> 32-bit
//			<o0.0> Channel length (number of bits per audio channel) (CHLEN) 
//				<0=> 16-bit wide
//				<1=> 32-bit wide
//		</h>

#define SPI2_I2SCFGR_VALUE		0x00000000

//		<h> SPI_I2S prescaler register (SPI_I2SPR)
//			<o0.9> Master clock output enable (MCKOE)
//			<o0.8> Odd factor for the prescaler (ODD)
//				<0=> real divider value is = I2SDIV *2
//				<1=> real divider value is = (I2SDIV * 2)+1
//			<o0.0..7> I2S Linear prescaler (I2SDIV)  <2-127> 
//		</h>
//	</e>

#define SPI2_I2SPR_VALUE		0x00000002

// </h> SPI2

// <h> SPI3
//	<o2.11> I2S mode selection (I2SMOD)
//		<0=> SPI mode
//		<1=> I2S mode
//	<e0.6> SPI Mode
//		<h> SPI control register 1 (SPI_CR1)
//			<o0.15> Bidirectional data mode enable (BIDIMODE)
//				<0=> 2-line unidirectional
//				<1=> 1-line bidirectional
//			<o0.14> Output enable in bidirectional mode (BIDIOE)
//				<0=> Output disabled (receive-only mode)
//				<1=> Output enabled (transmit-only mode)
//			<o0.13> Hardware CRC calculation enable (CRCEN)
//			<o0.12> CRC transfer next (CRCNEXT)
//				<0=> Data phase (no CRC phase)
//				<1=> Next transfer is CRC (CRC phase)
//			<o0.11> Data frame format(DFF)
//				<0=> 8-bit 
//				<1=> 16-bit
//			<o0.10> Receive only (RXONLY)
//				<0=> Full duplex (Transmit and receive)
//				<1=> Output disabled (Receive-only mode)
//			<o0.9> Software slave management (SSM)
//				<0=> Software slave management disabled
//				<1=> Software slave management enabled
//			<o0.7>Frame format (LSBFIRST)
//				<0=> MSB transmitted first
//				<1=> LSB transmitted first
//			<o0.3..5> Baud rate control (BR[2:0])
//				<0=>Fpclk/2
//				<1=>Fpclk/4
//				<2=>Fpclk/8
//				<3=>Fpclk/16
//				<4=>Fpclk/32 
//				<5=>Fpclk/64
//				<6=>Fpclk/128
//				<7=>Fpclk/256
//			<o0.2> Master selection (MSTR)
//				<0=> Slave
//				<1=> Master
//			<o0.1> Clock polarity (CPOL)
//				<0=> CK to 0 when idle
//				<1=> CK to 1 when idle
//			<o0.0> Clock phase (CPHA)
//				<0=> First edge Capture 
//				<1=> Second edge Capture
//		</h>


#define SPI3_CR1_VALUE		0x00000000

//		<h> SPI control register 2 (SPI_CR2)
//			<o0.7> Tx buffer empty interrupt enable (TXEIE)
//			<o0.6> RX buffer not empty interrupt enable (RXNEIE)
//			<o0.5> Error interrupt enable (ERRIE)
//			<o0.4> Frame format (FRF)
//				<0=> SPI Motorola mode
//				<1=> SPI TI mode
//			<o0.2> SS output enable (SSOE)
//			<o0.1> Tx buffer DMA enable (TXDMAEN)
//			<o0.0> Rx buffer DMA enable (RXDMAEN)
//		</h>
//	</e>
#define SPI3_CR2_VALUE		0x00000000

//	<e0.10> I2S Mode
//		<h> SPI_I2S configuration register (SPI_I2SCFGR)
//			<o0.8..9> I2S configuration mode (I2SCFG)
//				<0=> Slave - transmit
//				<1=> Slave - receive
//				<2=> Master - transmit
//				<3=> Master - receive
//			<o0.7> PCM frame synchronization (PCMSYNC)
//				<0=> Short frame synch
//				<1=> Long frame synch
//			<o0.4..5> I2S standard selection (I2SSTD)
//				<0=> I2S Philips standard.
//				<1=> MSB justified standard (left justified)
//				<2=> LSB justified standard (right justified)
//				<3=> PCM standard
//			<o0.3> Steady state clock polarity (CKPOL)
//				<0=> CK to 0 when idle
//				<1=> CK to 1 when idle
//			<o0.1..2> Data length to be transferred (DATLEN)
//				<0=> 16-bit
//				<1=> 24-bit
//				<2=> 32-bit
//			<o0.0> Channel length (number of bits per audio channel) (CHLEN) 
//				<0=> 16-bit wide
//				<1=> 32-bit wide
//		</h>

#define SPI3_I2SCFGR_VALUE		0x00000000

//		<h> SPI_I2S prescaler register (SPI_I2SPR)
//			<o0.9> Master clock output enable (MCKOE)
//			<o0.8> Odd factor for the prescaler (ODD)
//				<0=> real divider value is = I2SDIV *2
//				<1=> real divider value is = (I2SDIV * 2)+1
//			<o0.0..7> I2S Linear prescaler (I2SDIV)  <2-127> 
//		</h>
//	</e>

#define SPI3_I2SPR_VALUE		0x00000002

// </h> SPI3
//</h>

/******************************************************************************************************/

void SystemInit(void){
	
//-------------------- Flash Registers -----------------------
	
	/*Flash access control register */
	FLASH -> ACR = FLASH_ACR_VALUE;
	
	FlashCRLock(False);
	FLASH -> CR = FLASH_CR_VALUE;
	FlashCRLock(True);
//-------------------- Power Registers -----------------------
	
	/*Power Control Register*/
	PWR -> CR |= PWR_CR_VALUE;
	/*PWR power control/status register*/
	PWR -> CSR |= PWR_CSR_VALUE;

//------------------- SYSCFG registers ------------------------

	/*SYSCFG memory remap register*/
	SYSCFG -> MEMRMP = SYSCFG_MEMRMP_VALUE;
	/*SYSCFG peripheral mode configuration register*/
	SYSCFG -> PMC = SYSCFG_PMC_VALUE;
	/*SYSCFG external interrupt configuration register 1*/
	SYSCFG -> EXTICR[0] = SYSCFG_EXTICR1_VALUE; 
	/*SYSCFG external interrupt configuration register 1*/
	SYSCFG -> EXTICR[1] = SYSCFG_EXTICR2_VALUE; 
	/*SYSCFG external interrupt configuration register 1*/
	SYSCFG -> EXTICR[2] = SYSCFG_EXTICR3_VALUE; 
	/*SYSCFG external interrupt configuration register 1*/
	SYSCFG -> EXTICR[3] = SYSCFG_EXTICR4_VALUE;
	/*Compensation cell control register*/
	SYSCFG -> CMPCR = SYSCFG_CMPCR_VALUE;	
//---------------- Reset & Clock Registers -------------------

	/*RCC PLL configuration register*/
 	RCC -> PLLCFGR = RCC_PLLCFGR_VALUE;

	/*RCC clock configuration register*/
	RCC -> CFGR = RCC_CFGR_VALUE & 0xFFFFFFF8;
	while ((RCC -> CFGR & 0xFFFFFFF8) != (RCC_CFGR_VALUE & 0xFFFFFFF8)){};
	
	/* Reset Control Register */
	RCC -> CR = RCC_CR_VALUE & (RCC_CR_HSEON | RCC_CR_HSION);
	if ((RCC_CR_VALUE & RCC_CR_HSEON) == RCC_CR_HSEON)
		while(!(RCC -> CR & RCC_CR_HSERDY));

	if ((RCC_CR_VALUE & RCC_CR_HSION) == RCC_CR_HSION)
		while(!(RCC -> CR & RCC_CR_HSIRDY));
	
	RCC -> CR = RCC_CR_VALUE;
	if ((RCC_CR_VALUE & RCC_CR_PLLON) == RCC_CR_PLLON)
		while(!(RCC -> CR & RCC_CR_PLLRDY));
		
	if ((RCC_CR_VALUE & RCC_CR_PLLI2SON) == RCC_CR_PLLI2SON)
		while(!(RCC -> CR & RCC_CR_PLLI2SRDY)); 		
	
	/*RCC clock configuration register*/
	RCC -> CFGR |= RCC_CFGR_VALUE & 0x00000003;
	while ((RCC -> CFGR & 0x00000003) != (RCC_CFGR_VALUE & 0x00000003)){};	
	

	/*RCC clock interrupt register*/
	RCC -> CIR = RCC_CIR_VALUE;
	/*RCC AHB1 peripheral clock enable register*/
	RCC -> AHB1ENR = RCC_AHB1ENR_VALUE;
	/*RCC AHB2 peripheral clock enable register*/
	RCC -> AHB2ENR = RCC_AHB2ENR_VALUE;
	/*RCC AHB3 peripheral clock enable register*/
	RCC -> AHB3ENR = RCC_AHB3ENR_VALUE;
	/*RCC APB1 peripheral clock enable register*/
	RCC -> APB1ENR = RCC_APB1ENR_VALUE;
	/*RCC APB2 peripheral clock enable register*/
	RCC -> APB2ENR = RCC_APB2ENR_VALUE;
	/*RCC AHB1 peripheral clock enable in low power mode register*/
	RCC -> AHB1LPENR = RCC_AHB1LPENR_VALUE;
	/*RCC AHB2 peripheral clock enable in low power mode register*/
	RCC -> AHB2LPENR = RCC_AHB2LPENR_VALUE;
	/*RCC AHB3 peripheral clock enable in low power mode register*/
	RCC -> AHB3LPENR = RCC_AHB3LPENR_VALUE;
	/*RCC APB1 peripheral clock enable in low power mode register*/
	RCC -> APB1LPENR = RCC_APB1LPENR_VALUE;
	/*RCC APB2 peripheral clock enabled in low power mode register*/
	RCC -> APB2LPENR = RCC_APB2LPENR_VALUE;
	/*RCC Backup domain control register*/
	RCC -> BDCR = RCC_BDCR_VALUE;
	/*RCC clock control & status register*/
	RCC -> CSR = RCC_CSR_VALUE;
	/*RCC PLLI2S configuration register*/
	RCC -> PLLI2SCFGR = RCC_PLLI2SCFGR_VALUE;

//-------------------- GPIO Registers -------------------------	
	
	/*GPIO port A mode register*/
	GPIOA -> MODER = GPIOA_MODER_VALUE;
	/*GPIO port B mode register*/
	GPIOB -> MODER = GPIOB_MODER_VALUE;
	/*GPIO port C mode register*/
	GPIOC -> MODER = GPIOC_MODER_VALUE;
	/*GPIO port D mode register*/
	GPIOD -> MODER = GPIOD_MODER_VALUE;
	/*GPIO port E mode register*/
	GPIOE -> MODER = GPIOE_MODER_VALUE;
	/*GPIO port F mode register*/
	GPIOF -> MODER = GPIOF_MODER_VALUE;
	/*GPIO port G mode register*/
	GPIOG -> MODER = GPIOG_MODER_VALUE;
	/*GPIO port H mode register*/
	GPIOH -> MODER = GPIOH_MODER_VALUE;
	/*GPIO port I mode register*/
	GPIOI -> MODER = GPIOI_MODER_VALUE;	
	
	/*GPIO port A output type register*/
	GPIOA -> OTYPER = GPIOA_OTYPER_VALUE;
	/*GPIO port B output type register*/
	GPIOB -> OTYPER = GPIOB_OTYPER_VALUE;
	/*GPIO port C output type register*/
	GPIOC -> OTYPER = GPIOC_OTYPER_VALUE;
	/*GPIO port D output type register*/
	GPIOD -> OTYPER = GPIOD_OTYPER_VALUE;
	/*GPIO port E output type register*/
	GPIOE -> OTYPER = GPIOE_OTYPER_VALUE;
	/*GPIO port F output type register*/
	GPIOF -> OTYPER = GPIOF_OTYPER_VALUE;
	/*GPIO port G output type register*/
	GPIOG -> OTYPER = GPIOG_OTYPER_VALUE;
	/*GPIO port H output type register*/
	GPIOH -> OTYPER = GPIOH_OTYPER_VALUE;
	/*GPIO port I output type register*/
	GPIOI -> OTYPER = GPIOI_OTYPER_VALUE;
	
	/*GPIO port A output speed register*/
	GPIOA -> OSPEEDR = GPIOA_OSPEEDR_VALUE;
	/*GPIO port B output speed register*/
	GPIOB -> OSPEEDR = GPIOB_OSPEEDR_VALUE;
	/*GPIO port C output speed register*/
	GPIOC -> OSPEEDR = GPIOC_OSPEEDR_VALUE;
	/*GPIO port D output speed register*/
	GPIOD -> OSPEEDR = GPIOD_OSPEEDR_VALUE;
	/*GPIO port E output speed register*/
	GPIOE -> OSPEEDR = GPIOE_OSPEEDR_VALUE;
	/*GPIO port F output speed register*/
	GPIOF -> OSPEEDR = GPIOF_OSPEEDR_VALUE;
	/*GPIO port G output speed register*/
	GPIOG -> OSPEEDR = GPIOG_OSPEEDR_VALUE;
	/*GPIO port H output speed register*/
	GPIOH -> OSPEEDR = GPIOH_OSPEEDR_VALUE;
	/*GPIO port I output speed register*/
	GPIOI -> OSPEEDR = GPIOI_OSPEEDR_VALUE;
	
	/*GPIO port A pull-up/pull-down register*/
	GPIOA -> PUPDR = GPIOA_PUPDR_VALUE;
	/*GPIO port B pull-up/pull-down register*/
	GPIOB -> PUPDR = GPIOB_PUPDR_VALUE;
	/*GPIO port C pull-up/pull-down register*/
	GPIOC -> PUPDR = GPIOC_PUPDR_VALUE;
	/*GPIO port D pull-up/pull-down register*/
	GPIOD -> PUPDR = GPIOD_PUPDR_VALUE;
	/*GPIO port E pull-up/pull-down register*/
	GPIOE -> PUPDR = GPIOE_PUPDR_VALUE;
	/*GPIO port F pull-up/pull-down register*/
	GPIOF -> PUPDR = GPIOF_PUPDR_VALUE;
	/*GPIO port G pull-up/pull-down register*/
	GPIOG -> PUPDR = GPIOG_PUPDR_VALUE;
	/*GPIO port H pull-up/pull-down register*/
	GPIOH -> PUPDR = GPIOH_PUPDR_VALUE;
	/*GPIO port I pull-up/pull-down register*/
	GPIOI -> PUPDR = GPIOI_PUPDR_VALUE;
	
	/*GPIO A alternate function low register*/
	GPIOA -> AFR[0] = GPIOA_AFRL_VALUE;
	/*GPIO B alternate function low register*/
	GPIOB -> AFR[0] = GPIOB_AFRL_VALUE;
	/*GPIO C alternate function low register*/
	GPIOC -> AFR[0] = GPIOC_AFRL_VALUE;
	/*GPIO D alternate function low register*/
	GPIOD -> AFR[0] = GPIOD_AFRL_VALUE;
	/*GPIO E alternate function low register*/
	GPIOE -> AFR[0] = GPIOE_AFRL_VALUE;
	/*GPIO F alternate function low register*/
	GPIOF -> AFR[0] = GPIOF_AFRL_VALUE;
	/*GPIO G alternate function low register*/
	GPIOG -> AFR[0] = GPIOG_AFRL_VALUE;
	/*GPIO H alternate function low register*/
	GPIOH -> AFR[0] = GPIOH_AFRL_VALUE;
	/*GPIO I alternate function low register*/
	GPIOI -> AFR[0] = GPIOH_AFRL_VALUE;
	
	/*GPIO A alternate function high register*/
	GPIOA -> AFR[1] = GPIOA_AFRH_VALUE;
	/*GPIO B alternate function high register*/
	GPIOB -> AFR[1] = GPIOB_AFRH_VALUE;
	/*GPIO C alternate function high register*/
	GPIOC -> AFR[1] = GPIOC_AFRH_VALUE;
	/*GPIO D alternate function high register*/
	GPIOD -> AFR[1] = GPIOD_AFRH_VALUE;
	/*GPIO E alternate function high register*/
	GPIOE -> AFR[1] = GPIOE_AFRH_VALUE;
	/*GPIO F alternate function high register*/
	GPIOF -> AFR[1] = GPIOF_AFRH_VALUE;
	/*GPIO G alternate function high register*/
	GPIOG -> AFR[1] = GPIOG_AFRH_VALUE;
	/*GPIO H alternate function high register*/
	GPIOH -> AFR[1] = GPIOH_AFRH_VALUE;
	/*GPIO I alternate function high register*/
	GPIOI -> AFR[1] = GPIOH_AFRH_VALUE;
	
//------------------- USART & UART registers --------------------
	/*Baud rate register*/
	USART1 -> BRR = USART1_BRR_VALUE;
	/*Control register 1*/
	USART1 -> CR1 = USART1_CR1_VALUE;
	/*Control register 2*/
	USART1 -> CR2 = USART1_CR2_VALUE;
	/*Control register 3*/
	USART1 -> CR3 = USART1_CR3_VALUE;
	
	/*Baud rate register*/
	USART2 -> BRR = USART2_BRR_VALUE;
	/*Control register 1*/
	USART2 -> CR1 = USART2_CR1_VALUE;
	/*Control register 2*/
	USART2 -> CR2 = USART2_CR2_VALUE;
	/*Control register 3*/
	USART2 -> CR3 = USART2_CR3_VALUE;
	
	/*Baud rate register*/
	USART3 -> BRR = USART3_BRR_VALUE;
	/*Control register 1*/
	USART3 -> CR1 = USART3_CR1_VALUE;
	/*Control register 2*/
	USART3 -> CR2 = USART3_CR2_VALUE;
	/*Control register 3*/
	USART3 -> CR3 = USART3_CR3_VALUE;
	
	/*Baud rate register*/
	UART4 -> BRR = USART4_BRR_VALUE;
	/*Control register 1*/
	UART4 -> CR1 = USART4_CR1_VALUE;
	/*Control register 2*/
	UART4 -> CR2 = USART4_CR2_VALUE;
	/*Control register 3*/
	UART4 -> CR3 = USART4_CR3_VALUE;
	
	/*Baud rate register*/
	UART5 -> BRR = USART5_BRR_VALUE;
	/*Control register 1*/
	UART5 -> CR1 = USART5_CR1_VALUE;
	/*Control register 2*/
	UART5 -> CR2 = USART5_CR2_VALUE;
	/*Control register 3*/
	UART5 -> CR3 = USART5_CR3_VALUE;
	
	/*Baud rate register*/
	USART6 -> BRR = USART6_BRR_VALUE;
	/*Control register 1*/
	USART6 -> CR1 = USART6_CR1_VALUE;
	/*Control register 2*/
	USART6 -> CR2 = USART6_CR2_VALUE;
	/*Control register 3*/
	USART6 -> CR3 = USART6_CR3_VALUE;
	
//------------------- DMA registers --------------------
	/*DMA low interrupt flag clear register*/
	DMA1 -> LIFCR = 0x00000000;
	/*DMA high interrupt flag clear register*/
	DMA1 -> HIFCR = 0x00000000;
	/*DMA stream 0 configuration register*/
	DMA1_Stream0 -> CR = DMA1_S0CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 0 FIFO control register*/
	DMA1_Stream0 -> FCR = DMA1_S0FCR_VALUE;
	

	/*DMA stream 1 configuration register*/
	DMA1_Stream1 -> CR = DMA1_S1CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 1 FIFO control register*/
	DMA1_Stream1 -> FCR = DMA1_S1FCR_VALUE;
	

	/*DMA stream 2 configuration register*/
	DMA1_Stream2 -> CR = DMA1_S2CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 2 FIFO control register*/
	DMA1_Stream2 -> FCR = DMA1_S2FCR_VALUE;
	

	/*DMA stream 3 configuration register*/
	DMA1_Stream3 -> CR = DMA1_S3CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 3 FIFO control register*/
	DMA1_Stream3 -> FCR = DMA1_S3FCR_VALUE;
	

	/*DMA stream 4 configuration register*/
	DMA1_Stream4 -> CR = DMA1_S4CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 4 FIFO control register*/
	DMA1_Stream4 -> FCR = DMA1_S4FCR_VALUE;
	
	
	/*DMA stream 5 configuration register*/
	DMA1_Stream5 -> CR = DMA1_S5CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 5 FIFO control register*/
	DMA1_Stream5 -> FCR = DMA1_S5FCR_VALUE;
	

	/*DMA stream 6 configuration register*/
	DMA1_Stream6 -> CR = DMA1_S6CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 6 FIFO control register*/
	DMA1_Stream6 -> FCR = DMA1_S6FCR_VALUE;


	/*DMA stream 7 configuration register*/
	DMA1_Stream7 -> CR = DMA1_S7CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 7 FIFO control register*/
	DMA1_Stream7 -> FCR = DMA1_S7FCR_VALUE;
	
	
	
	/*DMA low interrupt flag clear register*/
	DMA2 -> LIFCR = 0x00000000;
	/*DMA high interrupt flag clear register*/
	DMA2 -> HIFCR = 0x00000000;
	/*DMA stream 0 configuration register*/
	DMA2_Stream0 -> CR = DMA2_S0CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 0 FIFO control register*/
	DMA2_Stream0 -> FCR = DMA2_S0FCR_VALUE;
	

	/*DMA stream 1 configuration register*/
	DMA2_Stream1 -> CR = DMA2_S1CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 1 FIFO control register*/
	DMA2_Stream1 -> FCR = DMA2_S1FCR_VALUE;
	

	/*DMA stream 2 configuration register*/
	DMA2_Stream2 -> CR = DMA2_S2CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 2 FIFO control register*/
	DMA2_Stream2 -> FCR = DMA2_S2FCR_VALUE;
	

	/*DMA stream 3 configuration register*/
	DMA2_Stream3 -> CR = DMA2_S3CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 3 FIFO control register*/
	DMA2_Stream3 -> FCR = DMA2_S3FCR_VALUE;
	

	/*DMA stream 4 configuration register*/
	DMA2_Stream4 -> CR = DMA2_S4CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 4 FIFO control register*/
	DMA2_Stream4 -> FCR = DMA2_S4FCR_VALUE;
	
	
	/*DMA stream 5 configuration register*/
	DMA2_Stream5 -> CR = DMA2_S5CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 5 FIFO control register*/
	DMA2_Stream5 -> FCR = DMA2_S5FCR_VALUE;
	

	/*DMA stream 6 configuration register*/
	DMA2_Stream6 -> CR = DMA2_S6CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 6 FIFO control register*/
	DMA2_Stream6 -> FCR = DMA2_S6FCR_VALUE;


	/*DMA stream 7 configuration register*/
	DMA2_Stream7 -> CR = DMA2_S7CR_VALUE & 0xFFFFFFFE;
	/*DMA stream 7 FIFO control register*/
	DMA2_Stream7 -> FCR = DMA2_S7FCR_VALUE;
	
//------------------- SPI & I2S registers --------------------

	/*SPI Control Register 1*/
	SPI1 -> CR1 = SPI1_CR1_VALUE & 0xFFFFFFBF;
	/*SPI control register 2*/
	SPI1 -> CR2 = SPI1_CR2_VALUE;
	/*SPI_I2S prescaler register*/	
	SPI1 -> I2SPR = SPI1_I2SPR_VALUE;
	/*SPI_I2S configuration register */
	SPI1 -> I2SCFGR = SPI1_I2SCFGR_VALUE;
	/*SPI Control Register 1*/
	SPI1 -> CR1 |= SPI1_CR1_VALUE & 0x00000040;
	
	/*SPI Control Register 1*/
	SPI2 -> CR1 = SPI2_CR1_VALUE & 0xFFFFFFBF;
	/*SPI control register 2*/
	SPI2 -> CR2 = SPI2_CR2_VALUE;
	/*SPI_I2S prescaler register*/	
	SPI2 -> I2SPR = SPI2_I2SPR_VALUE;
	/*SPI_I2S configuration register */
	SPI2 -> I2SCFGR = SPI2_I2SCFGR_VALUE;
	/*SPI Control Register 1*/
	SPI2 -> CR1 |= SPI2_CR1_VALUE & 0x00000040;
	
	/*SPI Control Register 1*/
	SPI3 -> CR1 = SPI3_CR1_VALUE & 0xFFFFFFBF;
	/*SPI control register 2*/
	SPI3 -> CR2 = SPI3_CR2_VALUE;
	/*SPI_I2S prescaler register*/	
	SPI3 -> I2SPR = SPI3_I2SPR_VALUE;
	/*SPI_I2S configuration register */
	SPI3 -> I2SCFGR = SPI3_I2SCFGR_VALUE;
	/*SPI Control Register 1*/
	SPI3 -> CR1 |= SPI3_CR1_VALUE & 0x00000040;

}

/******************************************************************************************************/
//DMA Configuration
void DmaConfig(	DMA_Stream_TypeDef *Dma, 		//Set DMA Stream Number Here
				uint32 PeripheralAddr, 			//Peripheral Address Specify Here
				uint32 Memory0Addr, 			//Memory 0 Address Specify Here
				uint32 Memory1Addr,				//Memory 1 Address Specify Here
				uint16 Size						//Size of DMA Beats Specify Here
)
{
	if (!(Dma -> CR & DMA_SxCR_EN)){
		Dma -> PAR = PeripheralAddr;
		Dma -> M0AR = Memory0Addr;
		Dma -> M1AR = Memory1Addr;
		Dma -> NDTR = Size;
	}
}

//-------------------------------------------------------------------------------------------------------

void DmaEnable(	DMA_Stream_TypeDef *Dma, 			//Set DMA Stream Number Here
				boolean Enable						// True = Enable or False = Disable
)
{
	if (Enable == True)
		Dma -> CR |= 0x00000001;
	else if (Enable == False)
		Dma -> CR &= 0xFFFFFFFE;
}





//------------- <<< end of configuration section >>> -----------------------
