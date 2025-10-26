#include <stdint.h>
#include "stm32l053xx.h"
// LCD 16x2 (modo 4 bits):
//        D4–D7 = PA8–PA11
//        E  = PA5
//        RS = PA4

// Display 7 segmentos (4 dígitos, multiplexado):
//        Segmentos a–g = PB0–PB6
//        Enable dígitos D1–D4 = PC5, PC6, PC8, PC9
//        → D1 = decenas de minuto (izquierda)
//        → D2 = unidades de minuto
//        → D3 = decenas de segundo
//        → D4 = unidades de segundo (derecha)

// Keypad reducido (1 fila, 3 columnas):
//        Fila = PC4  → entrada con pull-up interno
//        Columnas = PB7–PB9 → salidas controladas por ODR
//        → C1 = 30 min, C2 = 40 min, C3 = 50 min

// Motor del tambor (controlado por driver L298N):
//        IN1 = PC2     → Dirección 1 (sentido horario)
//        IN2 = PC3     → Dirección 2 (sentido antihorario)
//        ENA (PWM velocidad) = PA0 (TIM2_CH1)
//        → Control de velocidad mediante PWM (Timer 2 Canal 1)
//        → Inversión de rotación controlada por software usando IN1/IN2

// Buzzer (alertas de inicio, fin, error):
//        PWM salida = PA6

// LEDs indicadores de etapa del ciclo:
//        Lavado     = PA12
//        Enjuague   = PA15
//        Centrifuga = PB10

// Botones de control (interrupciones EXTI):
//        Iniciar  = PC1  (EXTI1)  → Botón START del ciclo
//        Cancelar = PB11 (EXTI11) → Cancela ciclo actual

// Switch de tapa de seguridad:
//        Tapa = PB12 (EXTI12) → Bloquea arranque si está abierta

// USART2 (para monitoreo serial o módulo LTE/GSM):
//        TX = PA2  (USART2_TX)
//        RX = PA3  (USART2_RX)


void system_init(){
	//1. HSI 16Mhz
	RCC->CR |= (1<<0); //Encenderlo
	RCC->CFGR |= (1<<0); //Como clk del sistema

	//2. Clock GPIO's A,B,C
	RCC->IOPENR |= (1<<0) | (1<<1) | (1<<2);

	//3. Configuración Puertos (Keypad, Displays, Lcd, Leds, Buzzer, Push, Switch, Motor)

	//Leds salida (PA12, PA15, PB7)
	GPIOA->MODER &= ~((3 << (12 * 2)) | (3<<(15 * 2)));
	GPIOA->MODER |=  ((1 << (12 * 2)) | (1<<(15 * 2)));
	GPIOB->MODER &= ~(3 << (10 * 2));
	GPIOB->MODER |=  (1 << (10 * 2));

	//buzzer salida
	GPIOA->MODER &= ~(3 << (6 * 2));
	GPIOA->MODER |=  (1 << (6 * 2));

	// Push Buttons entrada y switch
	GPIOC->MODER &= ~(3 << (1 * 2));
	GPIOB->MODER &= ~((3 << (11 * 2)) | (3<<(12 * 2)));

	// Activar pull-up internos en los botones
	GPIOC->PUPDR &= ~(3u << (1 * 2));
	GPIOC->PUPDR |=  (1u << (1 * 2));   // 01 = pull-up

	GPIOB->PUPDR &= ~(3u << (11 * 2));
	GPIOB->PUPDR |=  (1u << (1 * 2));

	GPIOB->PUPDR &= ~(3u << (12 * 2));  // Limpia
	GPIOB->PUPDR |=  (2u << (12 * 2));  // 10 = Pull-down

	//Motor (salida) IN1 e IN2
	GPIOC->MODER &= ~((3<<(2 * 2)) | (3<<(3 * 2)));
	GPIOC->MODER |= ((1<<(2 * 2)) | (1<<(3 * 2)));

	//Enabled motor
	GPIOA->MODER &= ~(3 << (0*2));
	GPIOA->MODER |=  (2 << (0*2));       // Modo alternativo
	GPIOA->AFR[0] &= ~(0xF << (0*4));  // Limpia los 4 bits del AF de PA0
	GPIOA->AFR[0] |=  (2 << (0*4));   // Asigna AF2 → TIM2_CH1


	//4. Timers

	//Tim2 para motor PA0
	RCC->APB1ENR |= (1<<0); //Habilitar el timer
	TIM2->PSC = 16 - 1;   // 1 MHz
	TIM2->ARR = 50 - 1;   // 1 MHz / 50 = 20 kHz
	TIM2->CCR1 = 21;			// 50% duty inicial
	TIM2->CCMR1 &= ~(7u << 4);			//limpio el modo
	TIM2->CCMR1 |=  (6u << 4);          // PWM Mode 1 que es 110 = 6
	TIM2->CCER  |=  (1u << 0);            // Habilita salida CH1
	TIM2->CNT = 0;	// donde inicia el conteo
	TIM2->CR1 |= (1<<0);	// activar conteo



	//5. Reinicio de todo
	GPIOB->ODR &= ~((1 << 8) | (1 << 9)); //Apago motor
}

//Variables globales
uint8_t timer_sentido = 0;
uint8_t ticks_segundos = 0;
int8_t sentido = -1;

//Funciones del motor

// ============================
// FUNCIONES DEL MOTOR
// ============================

void lavado(void)
{
    TIM2->CCR1 = 21;
    GPIOC->ODR |=  (1 << 2);   // IN1 = 1
    GPIOC->ODR &= ~(1 << 3);   // IN2 = 0 → giro horario
    sentido = 0;               // sentido fijo
}

void enjuague(void)
{
    TIM2->CCR1 = 25;

    // Cada 3 ticks (ej. 300 ms, dependiendo del timer)
    if ((ticks_segundos - timer_sentido) >= 3)
    {
        timer_sentido = ticks_segundos;  // guarda referencia de tiempo
        sentido ^= 1;                    // alterna dirección

        if (sentido == 0) {
            GPIOC->ODR |=  (1 << 2);   // IN1 = 1
            GPIOC->ODR &= ~(1 << 3);   // IN2 = 0 (CW)
        } else {
            GPIOC->ODR &= ~(1 << 2);   // IN1 = 0
            GPIOC->ODR |=  (1 << 3);   // IN2 = 1 (CCW)
        }
    }
}

void centrifugado(void)
{
    TIM2->CCR1 = 30;      // velocidad alta (~80%)
    GPIOC->ODR |=  (1 << 2);   // IN1 = 1
    GPIOC->ODR &= ~(1 << 3);   // IN2 = 0 → sentido horario
    sentido = 0;
}


void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 1600; i++)  // ajusta según tu reloj (aprox. 16 MHz)
        __NOP();
}

int main(){
	system_init();
	while(1){
        // 🔹 Encender LED
        GPIOA->ODR |= (1u << 12);
        GPIOA->ODR |= (1u << 15);
        GPIOB->ODR |= (1u << 10);
        delay_ms(100);

        if (!(GPIOC->IDR & (1u << 1)) ||
            !(GPIOB->IDR & (1u << 12)))
        {
            GPIOA->ODR |= (1u << 6);  // buzzer ON
            GPIOC->ODR |= (1 << 2);
            GPIOC->ODR &= ~(1 << 3);
        }else if(!(GPIOB->IDR & (1u << 11))){
            GPIOC->ODR |= (1 << 3);
            GPIOC->ODR &= ~(1 << 2);
        }
        else
        {
            GPIOA->ODR &= ~(1u << 6);  // buzzer OFF
            GPIOC->ODR &= ~((1 << 2) | (1 << 3)); //Apago motor
        }



        // 🔹 Apagar LED
        GPIOA->ODR &= ~(1u << 12);
        GPIOA->ODR &= ~(1u << 15);
        GPIOB->ODR &= ~(1u << 10);
        delay_ms(100);

	}
}


