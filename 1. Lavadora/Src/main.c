#include <stdint.h>
#include "stm32l053xx.h"

// <-------Definición de pines------->

// LCD 16x2 (modo 4 bits):
//        D4–D7 = PA8–PA11
//        E  = PA5
//        RS = PA4

// Display 7 segmentos (4 dígitos, multiplexado):
//        Segmentos a–g, dp = PC0–PC7
//        Enable dígitos D1–D4 = PB10–PB13

// Keypad 4x4 (selección de ciclos y funciones):
//        Filas = PB0–PB3  → salidas
//        Columnas = PB4–PB7 → entradas con pull-up interno

// Motor del tambor (DC controlado por driver H-bridge):
//        IN1–IN2 = PC8–PC9
//        IN3–IN4 = PB8–PB9
//        PWM velocidad = TIM3_CH1–CH2
//        Inversión de rotación controlada por software

// Buzzer (alertas de inicio, fin, error):
//        PWM salida = PA0  (TIM2_CH1)

// LEDs indicadores de etapa del ciclo:
//        Lavado     = PC10
//        Enjuague   = PC11
//        Centrifuga = PC12

// Botones de control:
//        Iniciar  = PC13 (EXTI13)
//        Cancelar = PC14 (EXTI14)

// Switch de tapa de seguridad:
//        Tapa = PC15 (EXTI15) → bloquea arranque si está abierta

// USART2 (módulo LTE/GSM o monitoreo serial):
//        TX = PA2
//        RX = PA3

void initwasher(void)
{
    // 1) Habilitar reloj base (HSI16)
    RCC->CR |= (1 << 0);
    while(!(RCC->CR & (1 << 2))); // Esperar a que esté listo
    RCC->CFGR &= ~(0x3);          // HSI16 como SYSCLK
    RCC->CFGR |=  (0x1 << 0);

    // 2) Clocks GPIO
    RCC->IOPENR |= (1 << 0) | (1 << 1) | (1 << 2); // GPIOA, GPIOB, GPIOC
    RCC->APB2ENR |= (1u<<0);                   // SYSCFG

    // 3) Config GPIO's

    // LCD: PA4 (RS), PA5 (E), PA8–PA11 (D4–D7)
    GPIOA->MODER &= ~((3<<(4*2))|(3<<(5*2))|(3<<(8*2))|(3<<(9*2))|(3<<(10*2))|(3<<(11*2)));
    GPIOA->MODER |=  ((1<<(4*2))|(1<<(5*2))|(1<<(8*2))|(1<<(9*2))|(1<<(10*2))|(1<<(11*2)));

    // Buzzer PWM: PA0 (AF5 → TIM2_CH1)
    GPIOA->MODER &= ~(3<<(0*2));
    GPIOA->MODER |=  (2<<(0*2));       // Alternate Function
    GPIOA->AFR[0]  |=  (5<<(0*4));     // AF5

    // USART2 TX/RX: PA2 (TX), PA3 (RX) → AF4
    GPIOA->MODER &= ~((3<<(2*2))|(3<<(3*2)));
    GPIOA->MODER |=  ((2<<(2*2))|(2<<(3*2)));
    GPIOA->AFR[0]  |=  ((4<<(2*4))|(4<<(3*4)));


    //GPIOB

    // Keypad filas (PB0–PB3) → salidas
    GPIOB->MODER &= ~((3<<(0*2))|(3<<(1*2))|(3<<(2*2))|(3<<(3*2)));
    GPIOB->MODER |=  ((1<<(0*2))|(1<<(1*2))|(1<<(2*2))|(1<<(3*2)));

    // Keypad columnas (PB4–PB7) → entradas con pull-up
    GPIOB->MODER &= ~((3<<(4*2))|(3<<(5*2))|(3<<(6*2))|(3<<(7*2)));
    GPIOB->PUPDR &= ~((3<<(4*2))|(3<<(5*2))|(3<<(6*2))|(3<<(7*2)));
    GPIOB->PUPDR |=  ((1<<(4*2))|(1<<(5*2))|(1<<(6*2))|(1<<(7*2)));

    // Motor IN3–IN4 (PB8–PB9) → salidas
    GPIOB->MODER &= ~((3<<(8*2))|(3<<(9*2)));
    GPIOB->MODER |=  ((1<<(8*2))|(1<<(9*2)));

    // Display enable (PB10–PB13) → salidas
    GPIOB->MODER &= ~((3<<(10*2))|(3<<(11*2))|(3<<(12*2))|(3<<(13*2)));
    GPIOB->MODER |=  ((1<<(10*2))|(1<<(11*2))|(1<<(12*2))|(1<<(13*2)));


    //GPIOC

    // Display segmentos (PC0–PC7) → salidas
    GPIOC->MODER &= ~(
        (3<<(0*2))|(3<<(1*2))|(3<<(2*2))|(3<<(3*2))|
        (3<<(4*2))|(3<<(5*2))|(3<<(6*2))|(3<<(7*2))
    );
    GPIOC->MODER |= (
        (1<<(0*2))|(1<<(1*2))|(1<<(2*2))|(1<<(3*2))|
        (1<<(4*2))|(1<<(5*2))|(1<<(6*2))|(1<<(7*2))
    );

    // Motor IN1–IN2 (PC8–PC9) → salidas
    GPIOC->MODER &= ~((3<<(8*2))|(3<<(9*2)));
    GPIOC->MODER |=  ((1<<(8*2))|(1<<(9*2)));

    // LEDs de etapa (PC10–PC12) → salidas
    GPIOC->MODER &= ~((3<<(10*2))|(3<<(11*2))|(3<<(12*2)));
    GPIOC->MODER |=  ((1<<(10*2))|(1<<(11*2))|(1<<(12*2)));

    // Botones y tapa (PC13–PC15) → entradas con pull-up
    GPIOC->MODER &= ~((3<<(13*2))|(3<<(14*2))|(3<<(15*2)));
    GPIOC->PUPDR &= ~((3<<(13*2))|(3<<(14*2))|(3<<(15*2)));
    GPIOC->PUPDR |=  ((1<<(13*2))|(1<<(14*2))|(1<<(15*2)));

    GPIOA->ODR = 0x0000;
    GPIOB->ODR = 0x0000;
    GPIOC->ODR = 0x0000;

    // 4) EXIT (Flanco bajada)
    // Keypad (filas) y Botones (PC13, PC15


}

int main (void){
	while(1){

	}
}
