#include "stm32l4xx.h"
#include "stm32l4xx_nucleo.h"
#include "core_cm4.h"
#include <stdint.h>
// User LED = LD2 Green LED = PA.5
#define LED_PIN 5
// Push button = PC 13
#define EXTI_PIN 13

volatile uint32_t delay_time = 0; // delay time in milliseconds
volatile uint32_t button_press_time = 0; // to track button press duration
volatile uint8_t frequency_state = 0; // to cycle through frequencies
volatile uint8_t led_status = 0; // LED status

void LED_Init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    GPIOA->MODER &= ~(3U<<(2*LED_PIN));
    GPIOA->MODER |= 1U<<(2*LED_PIN);
    GPIOA->OSPEEDR &= ~(3U<<(2*LED_PIN));
    GPIOA->OSPEEDR |= 3U<<(2*LED_PIN);
    GPIOA->OTYPER &= ~(1U<<LED_PIN);
    GPIOA->PUPDR &= ~(3U<<(2*LED_PIN));
}

void LED_On(void) {
    GPIOA->ODR |= (1UL<<LED_PIN);
}

void LED_Off(void) {
    GPIOA->ODR &= ~(1UL<<LED_PIN);
}

void EXTI_Init(void) {
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
    GPIOC->MODER &= ~(3U<<(2*EXTI_PIN));
    GPIOC->PUPDR &= ~(3U<<(2*EXTI_PIN));

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;

    EXTI->IMR1 |= EXTI_IMR1_IM13;
    EXTI->RTSR1 |= EXTI_RTSR1_RT13;

    NVIC_EnableIRQ(EXTI15_10_IRQn);
    NVIC_SetPriority(EXTI15_10_IRQn, 0);
}

void SysTick_Init(uint32_t tick) {
    SysTick->LOAD = tick - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) {
    if (led_status == 1 && delay_time > 0) {
        static uint32_t ticks = 0;
        if (++ticks >= delay_time) {
            GPIOA->ODR ^= (1UL << LED_PIN);
            ticks = 0;
        }
    }

    if (button_press_time > 0) {
        button_press_time++;
    }
}

void EXTI15_10_IRQHandler(void) {
    if ((EXTI->PR1 & EXTI_PR1_PIF13) == EXTI_PR1_PIF13) {
        EXTI->PR1 |= EXTI_PR1_PIF13; // Clear interrupt flag

        if (button_press_time == 0) { // Button pressed
            button_press_time = 1;
        } else if (button_press_time > 1000) { // Button held for more than 1 second
            button_press_time = 0;
            frequency_state = 0;
            delay_time = 0;
            led_status = 0;
            LED_Off();
        } else { // Button released quickly
            button_press_time = 0;
            led_status = 1;
            frequency_state = (frequency_state + 1) % 4;
            delay_time = 500 * (frequency_state + 1); // 500ms, 1000ms, 1500ms, 2000ms
        }
    }
}

int main(void) {
    LED_Init();
    EXTI_Init();
    SysTick_Init(SystemCoreClock / 1000); // 1 ms tick

    while (1) {
        // Main loop can be empty if all control is done via interrupts
    }
}
