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

// Keypad reducido (1 fila, 3 columnas):
//        Fila = PB0  → entrada con pull-up interno
//        Columnas = PB4–PB6 → salidas controladas por ODR

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
//        Cancelar = PB1 (EXTI14)

// Switch de tapa de seguridad:
//        Tapa = PB2 (EXTI15) → bloquea arranque si está abierta

// USART2 (módulo LTE/GSM o monitoreo serial):
//        TX = PA2
//        RX = PA3

//Variables

//Variables Keypad
volatile uint8_t ciclo = 0;
volatile uint8_t ciclo_activo = 0;
volatile uint8_t fsm_keypad = 0;

//Variables de botones
volatile uint8_t flag_inicio = 0;
volatile uint8_t flag_tapadera = 0;
volatile uint8_t flag_cancelar = 0;
volatile uint8_t flag_tapa_abierta = 0;
volatile uint8_t fsm_lavadora = 0;

//<-------Keypad reducido------->
// 1 fila (PB0) y 3 columnas (PB4–PB6)

void tecla_activa(void)
{
    // Si ya hay un ciclo activo, no permitir seleccionar otro
    if (ciclo_activo == 1) {
        return;
    }

    // Escaneo simple de 3 columnas y 1 fila
    GPIOB->ODR |= (1<<4)|(1<<5)|(1<<6); // Todas las columnas HIGH

    GPIOB->ODR &= ~(1<<4);               // Activa columna 1
    if ( (GPIOB->IDR & (1<<0)) == 0 ) {  // Si fila PB0 lee bajo
        ciclo = 1;                       // Tecla '1'
    }
    GPIOB->ODR |= (1<<4);

    GPIOB->ODR &= ~(1<<5);               // Activa columna 2
    if ( (GPIOB->IDR & (1<<0)) == 0 ) {
        ciclo = 2;                       // Tecla '2'
    }
    GPIOB->ODR |= (1<<5);

    GPIOB->ODR &= ~(1<<6);               // Activa columna 3
    if ( (GPIOB->IDR & (1<<0)) == 0 ) {
        ciclo = 3;                       // Tecla '3'
    }
    GPIOB->ODR |= (1<<6);
}


// <-------Funciones de control de botones------->

// Botón INICIAR
void btn_start(void)
{
	if ((ciclo != 0) && (ciclo_activo == 0) && (flag_tapa_abierta == 0)) {
	    // Arrancar ciclo
	    flag_inicio = 1;
	}
	else if((ciclo == 0) && (ciclo_activo == 0) && (flag_tapa_abierta == 0)){
		// lcd_print("Seleccione ciclo");
		flag_inicio = 0;
	}
	else if((ciclo != 0) && (ciclo_activo == 1) && (flag_tapa_abierta == 0)){
		// lcd_print("Ciclo en proceso");
	}
	else if((ciclo != 0) && (ciclo_activo == 0) && (flag_tapa_abierta == 1)){
		// lcd_print("Cerrar tapa");
		flag_inicio = 0;
	}
}

// Botón CANCELAR
void btn_cancelar(void)
{
    // Solo cancelar si hay un ciclo activo o seleccionado
    if ((ciclo != 0) || (ciclo_activo == 1)) {

        ciclo = 0;
        ciclo_activo = 0;
        flag_inicio = 0;
        fsm_lavadora = 0;   // estado IDLE

        // Apagar LEDs de etapa
        GPIOC->ODR &= ~((1<<10)|(1<<11)|(1<<12));

        // Detener motor (más adelante agregás la función)
        // motor_stop();

        // lcd_print("Ciclo cancelado");
        // USART2_Putstring((uint8_t*)"Ciclo cancelado\r\n");
    }
}

// Switch de tapa (seguridad)
void leer_tapa(void)
{
    if ((GPIOB->IDR & (1<<2)) == 0) {
        flag_tapa_abierta = 1;   // tapa abierta (lee LOW)
    } else {
        flag_tapa_abierta = 0;   // tapa cerrada
    }
}

void tipo_ciclo(){
	if((flag_inicio = 1) && (flag_tapadera_abierta == 0)){

	}
}


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

    // Keypad fila (PB0) → entrada con pull-up interno
    GPIOB->MODER &= ~(3<<(0*2));
    GPIOB->PUPDR &= ~(3<<(0*2));
    GPIOB->PUPDR |=  (1<<(0*2));

    // Keypad columnas (PB4–PB6) → salidas controladas por ODR
    GPIOB->MODER &= ~((3<<(4*2))|(3<<(5*2))|(3<<(6*2)));
    GPIOB->MODER |=  ((1<<(4*2))|(1<<(5*2))|(1<<(6*2)));
    GPIOB->ODR   |=  ((1<<4)|(1<<5)|(1<<6)); // arranque HIGH


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

    // Botones y tapa (PC13,PB1,PB2) → entradas
    GPIOC->MODER &= ~(3<<(13*2));
    GPIOB->MODER &= ~(3<<(1*2));
    GPIOB->MODER &= ~(3<<(2*2));
    GPIOB->PUPDR &= ~(3<<(2*2));
    GPIOB->PUPDR |=  (1<<(2*2));   // pull-up interno

    GPIOA->ODR = 0x0000;
    GPIOB->ODR = 0x0000;
    GPIOC->ODR = 0x0000;

    // 4) EXIT (Flanco bajada)
    // Keypad (filas) y Botones (PC13, PB1, PB2)
}


int main (void){
	initwasher();
	while(1){

	}

}
