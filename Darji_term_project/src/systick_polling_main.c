/**
  ******************************************************************************
  * @file    systick_polling_main.c
  * @author  Jian Zhang
  * @version V1.0
  * @date    10-31-2022
  * @brief   an example to use systick to implement an delay function,
  * 	     By this delay function to flash the LED:
  * 	     1. by default every 0.5s to toggle the LED,
  * 	     2. press the user button once, the LED toggle time will add with 0.5s
  * 	     3. IF the toggle reach to 2s, if press the button, the toggle time will set back to 0.5s
  ******************************************************************************
*/


#include "stm32l4xx.h"
#include "stm32l4xx_nucleo.h"
			

// User LED = LD2 Green LED = PA.5
#define LED_PIN 5
// Push button = PC 13
#define EXTI_PIN 13

volatile uint32_t delay_time;	//must be define as volatile
volatile uint32_t msTicks;		//count the number of systick interrupts

//******************************************************************************************
// Switch the PLL source from MSI to HSI, and select the PLL as SYSCLK source.
// to set the system clock to 80M HZ
//******************************************************************************************
void System_Clock_Init(void){

	uint32_t HSITrim;

	// To correctly read data from FLASH memory, the number of wait states (LATENCY)
  // must be correctly programmed according to the frequency of the CPU clock
  // (HCLK) and the supply voltage of the device.
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;

	// Enable the Internal High Speed oscillator (HSI
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	// Adjusts the Internal High Speed oscillator (HSI) calibration value
	// RC oscillator frequencies are factory calibrated by ST for 1 % accuracy at 25oC
	// After reset, the factory calibration value is loaded in HSICAL[7:0] of RCC_ICSCR
	HSITrim = 16; // user-programmable trimming value that is added to HSICAL[7:0] in ICSCR.
	RCC->ICSCR &= ~RCC_ICSCR_HSITRIM;
	RCC->ICSCR |= HSITrim << 24;

	RCC->CR    &= ~RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) == RCC_CR_PLLRDY);

	// Select clock source to PLL
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // 00 = No clock, 01 = MSI, 10 = HSI, 11 = HSE

	// Make PLL as 80 MHz
	// f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 16MHz * 20/2 = 160 MHz
	// f(PLL_R) = f(VCO clock) / PLLR = 160MHz/2 = 80MHz
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 20U << 8;
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 1U << 4; // 000: PLLM = 1, 001: PLLM = 2, 010: PLLM = 3, 011: PLLM = 4, 100: PLLM = 5, 101: PLLM = 6, 110: PLLM = 7, 111: PLLM = 8

	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLR;  // 00: PLLR = 2, 01: PLLR = 4, 10: PLLR = 6, 11: PLLR = 8
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN; // Enable Main PLL PLLCLK output

	RCC->CR   |= RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) == 0);

	// Select PLL selected as system clock
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL; // 00: MSI, 01:HSI, 10: HSE, 11: PLL

	// Wait until System Clock has been selected
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	// The maximum frequency of the AHB, the APB1 and the APB2 domains is 80 MHz.
	RCC->CFGR &= ~RCC_CFGR_HPRE;  // AHB prescaler = 1; SYSCLK not divided
	RCC->CFGR &= ~RCC_CFGR_PPRE1; // APB high-speed prescaler (APB1) = 1, HCLK not divided
	RCC->CFGR &= ~RCC_CFGR_PPRE2; // APB high-speed prescaler (APB2) = 1, HCLK not divided

	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
	// RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
	// RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN; // Enable Main PLL PLLSAI3CLK output enable
	// RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN; // Enable Main PLL PLL48M1CLK output enable

	RCC->CR &= ~RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
	while ( (RCC->CR & RCC_CR_PLLSAI1ON) == RCC_CR_PLLSAI1ON );

	// Configure and enable PLLSAI1 clock to generate 11.294MHz
	// 8 MHz * 24 / 17 = 11.294MHz
	// f(VCOSAI1 clock) = f(PLL clock input) *  (PLLSAI1N / PLLM)
	// PLLSAI1CLK: f(PLLSAI1_P) = f(VCOSAI1 clock) / PLLSAI1P
	// PLLUSB2CLK: f(PLLSAI1_Q) = f(VCOSAI1 clock) / PLLSAI1Q
	// PLLADC1CLK: f(PLLSAI1_R) = f(VCOSAI1 clock) / PLLSAI1R
	RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1N;
	RCC->PLLSAI1CFGR |= 24U<<8;

	// SAI1PLL division factor for PLLSAI1CLK
	// 0: PLLSAI1P = 7, 1: PLLSAI1P = 17
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1P;
	RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1PEN;

	// SAI1PLL division factor for PLL48M2CLK (48 MHz clock)
	// RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1Q;
	// RCC->PLLSAI1CFGR |= U<<21;
	// RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1QEN;

	// PLLSAI1 division factor for PLLADC1CLK (ADC clock)
	// 00: PLLSAI1R = 2, 01: PLLSAI1R = 4, 10: PLLSAI1R = 6, 11: PLLSAI1R = 8
	// RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1R;
	// RCC->PLLSAI1CFGR |= U<<25;
	// RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;

	RCC->CR |= RCC_CR_PLLSAI1ON;  // SAI1 PLL enable
	while ( (RCC->CR & RCC_CR_PLLSAI1ON) == 0);

	// SAI1 clock source selection
	// 00: PLLSAI1 "P" clock (PLLSAI1CLK) selected as SAI1 clock
	// 01: PLLSAI2 "P" clock (PLLSAI2CLK) selected as SAI1 clock
	// 10: PLL "P" clock (PLLSAI3CLK) selected as SAI1 clock
	// 11: External input SAI1_EXTCLK selected as SAI1 clock
	RCC->CCIPR &= ~RCC_CCIPR_SAI1SEL;

	RCC->APB2ENR |= RCC_APB2ENR_SAI1EN;
}


//******************************************************************************************
// User LED Configuration
//******************************************************************************************

//initialize LED
void LED_Init(void)
{
	// Enable the peripheral clock of GPIO Port
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOA->MODER &= ~(3U<<(2*LED_PIN));
	GPIOA->MODER |= 1U<<(2*LED_PIN);      //  Output(01)

	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	GPIOA->OSPEEDR &= ~(3U<<(2*LED_PIN));
	GPIOA->OSPEEDR |=   3U<<(2*LED_PIN);  // High speed

	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
	GPIOA->OTYPER &= ~(1U<<LED_PIN);       // Push-pull

	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOA->PUPDR   &= ~(3U<<(2*LED_PIN));  // No pull-up, no pull-down
}

// Turn LED On
void LED_On(void){
	GPIOA->ODR |= (1UL<<LED_PIN);
}

// Turn LED Off
void LED_Off(void){
	GPIOA->ODR &= ~(1UL<<LED_PIN);
}

// Toggle LED
void LED_Toggle(void)
{
	GPIOA->ODR ^= (1UL<<LED_PIN);
}

//******************************************************************************************
// User button Configuration with interrupt enabled
//******************************************************************************************
void User_button_EXTI_Init(void)
{
	// GPIO Configuration
	RCC->AHB2ENR |=   RCC_AHB2ENR_GPIOCEN;

	// GPIO Mode: Input(00, reset), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOC->MODER &= ~(3U<<(2*EXTI_PIN)); //inputs

	// GPIO PUDD: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOC->PUPDR &= ~(3U<<(2*EXTI_PIN)); // no pull-up, no pull down
	//GPIOC->PUPDR |=  (1U<<(2*EXTI_PIN));

	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)

	// EXIT Interrupt Enable
	NVIC_EnableIRQ(EXTI15_10_IRQn); 		//EXTI15_10_IRQn  WITH EXTI15_10_IRQHandler	Handler for pins connected to line 10 to 15
    NVIC_SetPriority(EXTI15_10_IRQn, 0);

	// Connect External Line to the GPIO
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;     // SYSCFG external interrupt configuration registers 70
	SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC;

	// Interrupt Mask Register (IMR)
	EXTI->IMR1 |= EXTI_IMR1_IM13;     // 0 = marked, 1 = not masked (i.e., enabled), enable bit 13

	// Software interrupt event register
	// EXTI->SWIER1 |= EXTI_SWIER1_SWI0;

	// Rising trigger selection register (RTSR)
	EXTI->RTSR1 |= EXTI_RTSR1_RT13;  // 0 = disabled, 1 = enabled

	// Falling trigger selection register (FTSR)
	//EXTI->FTSR1 |= EXTI_FTSR1_FT13;  // 0 = disabled, 1 = enabled
}


//******************************************************************************************
// Initialize SysTick
//******************************************************************************************
void SysTick_Init(void)
{
	// The RCC feeds the Cortex System Timer (SysTick) external clock with the AHB clock
	// (HCLK) divided by 8. The SysTick can work either with this clock or with the Cortex clock
	// (HCLK), configurable in the SysTick Control and Status Register.

	//  SysTick Control and Status Register
	SysTick->CTRL = 0;					 // Disable SysTick IRQ and SysTick Counter

	// SysTick Reload Value Register
	SysTick->LOAD = 80000000/1000 - 1;    // 1ms, Default clock, here we set the clock to 80MHz

	// SysTick Current Value Register
	SysTick->VAL = 0;					// write anything to VAL, making the counter restart from the SysTick_LOAD

	NVIC_SetPriority(SysTick_IRQn, 1);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable SysTick_IRQn interrupt in NVIC

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	// 0 = counting down to zero does not assert the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;		//Processor clock

	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

// SysTick Interrupt Handler
// by current systick with clock of 80M HZ, every 1ms will generate an interrupt
void SysTick_Handler(void)
{
	msTicks++;
}


//ISR function for button's interrupt
void EXTI15_10_IRQHandler(void)
{
	// NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	// PR: Pending register
	if ((EXTI->PR1 & EXTI_PR1_PIF13) == EXTI_PR1_PIF13) //ensure the interrupt is from EXTI13(by user button)
	{
		// cleared by writing a 1 to this bit
		EXTI->PR1 |= EXTI_PR1_PIF13;
		if(delay_time <= 2000)
		{
			delay_time += 500;	//add more 0.5s delay
		}
		else
		{
			delay_time = 500;
		}
	}
}


//******************************************************************************************
// Delay in ms
//******************************************************************************************
void delay (uint32_t T)
{
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < T);		//busy wait the curTicks > T

  msTicks = 0;
}


int main(void)
{
	System_Clock_Init(); // Set System Clock to: 80 MHz
	LED_Init();
	User_button_EXTI_Init();
	SysTick_Init();
	delay_time = 500; //initialize as delay 500ms (0.5s)

	while(1)
	{
		delay(delay_time); //wait for delay_time, then toggle the LED
		LED_Toggle();
	}
}
