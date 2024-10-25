/**
  ******************************************************************************
  * @file    exti-main.c
  * @author  Jian Zhang
  * @version V1.0
  * @date    10-27-2022
  * @brief   This is an example of enabling the EXTI13 for user button (PC13)
  *          It will capture the raising edge of PC13, and toggle the LED.
  *          Since release button will generate an raising edge, it will
  *          toggle the LED while we release the button
  ******************************************************************************
*/


#include "stm32l4xx.h"
#include "stm32l4xx_nucleo.h"


// Push button = PC.13
#define EXTI_PIN 13
// User LED = LD2 Green LED = PA.5
#define LED_PIN 5

/******************************************************************************************
 *  initial the USER LED, by configuring the GPIOA and Pin 5
*******************************************************************************************/
void LED_Init(void)
{
	// Enable the peripheral clock of GPIO Port A
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

//******************************************************************************************
// Turn LED On
//******************************************************************************************
void LED_On(void)
{
	GPIOA->ODR |= (1UL<<LED_PIN);
}

//******************************************************************************************
// Turn LED Off
//******************************************************************************************
void LED_Off(void)
{
	GPIOA->ODR &= ~(1UL<<LED_PIN);
}

//******************************************************************************************
// Toggle LED
//******************************************************************************************
void LED_Toggle(void)
{
	GPIOA->ODR ^= (1UL<<LED_PIN);
}



/******************************************************************************************
 *  initial the User button with EXTI interrupt
*******************************************************************************************/
void EXTI_Init(void)
{
	// GPIO Configuration of Button
	RCC->AHB2ENR |=   RCC_AHB2ENR_GPIOCEN;

	// GPIO Mode: Input(00, reset), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOC->MODER &= ~(3U<<(2*EXTI_PIN)); //inputs

	// GPIO PUDD: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOC->PUPDR &= ~(3U<<(2*EXTI_PIN)); // no pull-up, no pull down

	// EXIT Interrupt Enable
	NVIC_EnableIRQ(EXTI15_10_IRQn); 		//EXTI15_10_IRQn  WITH EXTI15_10_IRQHandler	Handler for pins connected to line 10 to 15
    NVIC_SetPriority(EXTI15_10_IRQn, 0);

	// Connect External Line to the GPIO
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;			 //eEnable the peripheral clock of SYSCFG
	//EXTICR[0]: EXTI0~3, EXTICR[1]: EXTI4~7, EXTICR[2]:EXTI8~11, EXTICR[3]:EXTI12~EXTI15
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;     //SYSCFG external interrupt configuration registers
	SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC;  //Bit4-7 for EXTI13, write "010"/"0010" to bit 4-7 connect PC13 to EXTI13

	// Interrupt Mask Register (IMR)
	EXTI->IMR1 |= EXTI_IMR1_IM13;     // 0 = marked, 1 = not masked (i.e., enabled), enable bit 13

	//Enable rising edge when we release the button, interrupt triggered
	// Rising trigger selection register (RTSR)
	EXTI->RTSR1 |= EXTI_RTSR1_RT13;  // 0 = disabled, 1 = enabled

	//while if enable falling edge when we push down the button, interrupt triggered
	// Falling trigger selection register (FTSR)
	//EXTI->FTSR1 |= EXTI_FTSR1_FT13;  // 0 = disabled, 1 = enabled
}

void EXTI15_10_IRQHandler(void)
{
	//NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	// PR: Pending register
	//EXTI15_10_IRQHandler will respond to EXTI 10, 11,12,13,14,15,
	//first check if EXTI 13 is triggered
	if ((EXTI->PR1 & EXTI_PR1_PIF13) == EXTI_PR1_PIF13)
	{
		// cleared by writing a 1 to this bit
		EXTI->PR1 |= EXTI_PR1_PIF13;
		//introduce delay to watch if the PR is not cleared,
		//this ISR will be repeatedly called thus LED flashed
		//for(int i=0; i <100000; i++);
		LED_Toggle();
	}
}


// Push blue use button and toggle RED LED
int main(void)
{
	LED_Init();
	EXTI_Init();
	while(1);
}
