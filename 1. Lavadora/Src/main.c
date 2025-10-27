#include <stdint.h>
#include <string.h>
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

//Variables fijas reloj
#define ALL_DISPLAY_OFF 0x33
#define D0_ctrl 		(1<<5)
#define D1_ctrl 		(1<<6)
#define D2_ctrl 		(1<<8)
#define D3_ctrl 		(1<<9)

#define cc_0 0b0111111
#define cc_1 0b0000110
#define cc_2 0b1011011
#define cc_3 0b1001111
#define cc_4 0b1100110
#define cc_5 0b1101101
#define cc_6 0b1111101
#define cc_7 0b0000111
#define cc_8 0b1111111
#define cc_9 0b1101111
#define cc_all_off 0b0000000

#define ca_cc_bits 0xFF //sirve más adelante para limpiar segmentos

//Variables globales

//A. Motor
uint8_t timer_sentido = 0;
uint8_t ticks_segundos = 0;
int8_t sentido = -1;
volatile uint16_t segundos_etapa = 0;
volatile uint8_t etapa = 0;


//B. Lcd
volatile uint8_t lcd_state = 0;
volatile uint8_t lcd_step = 0;
volatile uint32_t tick_ms = 0;
volatile uint8_t lcd_index = 0;
volatile uint8_t fin_ciclo_tick = 0;
const char *lcd_text = "Seleccione Ciclo";

// C. Variables internas del keypad (para debounce)
volatile uint8_t ciclo = 0;        // ciclo activo
volatile uint8_t ciclo_prev = 0;

//D. Display
volatile uint8_t display_clk[4] = {0,0,0,0};
volatile uint8_t en_control = 0;
volatile uint8_t tick_seg = 0;

//E. Control de clk inverso
volatile uint8_t ciclo_activo = 0;
volatile uint16_t tiempo_total = 0; // tiempo total en segundos

//F. Buzzer
volatile uint16_t buzzer_ms = 0;

void system_init(){
	//1. HSI 16Mhz
	RCC->CR |= (1<<0); //Encenderlo
	RCC->CFGR |= (1<<0); //Como clk del sistema

	//2. Clock GPIO's A,B,C
	RCC->IOPENR |= (1<<0) | (1<<1) | (1<<2);

	//3. Configuración Puertos (Keypad, Displays, Lcd, Leds, Buzzer, Push, Switch, Motor)

	// A. Keypad (1x3)
	// Fila PC4 entrada con pull-up
	GPIOC->MODER &= ~(3 << (4*2));
	GPIOC->PUPDR &= ~(3 << (4*2));
	GPIOC->PUPDR |=  (1 << (4*2)); // Pull-up activado

	// Columnas PB7–PB9 salidas
	GPIOB->MODER &= ~((3<<(7*2)) | (3<<(8*2)) | (3<<(9*2)));
	GPIOB->MODER |=  ((1<<(7*2)) | (1<<(8*2)) | (1<<(9*2)));

	// Inicializa columnas HIGH
	GPIOB->ODR |= (1<<7) | (1<<8) | (1<<9);

	//B. Display

	//A-G salida
	GPIOB->MODER &= ~((3<<(0 * 2)) | (3<<(1 * 2)) | (3<<(2 * 2)) | (3<<(3 * 2)) | (3<<(4 * 2)) | (3<<(5 * 2)) | (3<<(6 * 2)));
	GPIOB->MODER |= ((1<<(0 * 2)) | (1<<(1 * 2)) | (1<<(2 * 2)) | (1<<(3 * 2)) | (1<<(4 * 2)) | (1<<(5 * 2)) | (1<<(6 * 2)));

	//Enables D1-D4
	GPIOC->MODER &= ~((3<<(5 * 2)) | (3<<(6 * 2)) | (3<<(8 * 2)) | (3<<(9 * 2)));
	GPIOC->MODER |= ((1<<(5 * 2)) | (1<<(6 * 2)) | (1<<(8 * 2)) | (1<<(9 * 2)));

	//C. LCD
	GPIOA->MODER &= ~((3<<(4*2)) | (3<<(5*2)) | (3<<(8 * 2)) | (3<<(9 * 2)) | (3<<(10 * 2)) | (3<<(11 * 2)));
	GPIOA->MODER |= ((1<<(4*2)) | (1<<(5*2)) | (1<<(8 * 2)) | (1<<(9 * 2)) | (1<<(10 * 2)) | (1<<(11 * 2)));

	//D. Leds salida (PA12, PA15, PB7)
	GPIOA->MODER &= ~((3 << (12 * 2)) | (3<<(15 * 2)));
	GPIOA->MODER |=  ((1 << (12 * 2)) | (1<<(15 * 2)));
	GPIOB->MODER &= ~(3 << (10 * 2));
	GPIOB->MODER |=  (1 << (10 * 2));

	//E. buzzer salida
	GPIOA->MODER &= ~(3 << (6 * 2));
	GPIOA->MODER |=  (1 << (6 * 2));

	//F. Push Buttons entrada y switch
	GPIOC->MODER &= ~(3 << (1 * 2));
	GPIOB->MODER &= ~((3 << (11 * 2)) | (3<<(12 * 2)));


	// Activar pull-up internos en los botones
	GPIOC->PUPDR &= ~(3u << (1 * 2));
	GPIOC->PUPDR |=  (1u << (1 * 2));   // pull-up

	GPIOB->PUPDR &= ~(3u << (11 * 2));
	GPIOB->PUPDR |=  (1u << (1 * 2));	// pull-up

	GPIOB->PUPDR &= ~(3u << (12 * 2));
	GPIOB->PUPDR |=  (2u << (12 * 2));  // Pull-down

	//G. Motor (salida) IN1 e IN2
	GPIOC->MODER &= ~((3<<(2 * 2)) | (3<<(3 * 2)));
	GPIOC->MODER |= ((1<<(2 * 2)) | (1<<(3 * 2)));

	//Enabled motor
	GPIOA->MODER &= ~(3 << (0*2));
	GPIOA->MODER |=  (2 << (0*2));       // Modo alternativo
	GPIOA->AFR[0] &= ~(0xF << (0*4));  // Limpia los 4 bits del AF de PA0
	GPIOA->AFR[0] |=  (2 << (0*4));   // Asigna AF2 TIM2_CH1


	//4. Timers

	//A. Tim2 para motor PA0
	RCC->APB1ENR |= (1<<0); //Habilitar el timer
	TIM2->PSC = 16 - 1;   // 1 MHz
	TIM2->ARR = 50 - 1;   // 1 MHz / 50 = 20 kHz
	TIM2->CCR1 = 21;
	TIM2->CCMR1 &= ~(7u << 4);			//limpio el modo
	TIM2->CCMR1 |=  (6u << 4);          // PWM Mode 1 que es 110 = 6
	TIM2->CCER  |=  (1u << 0);            // Habilita salida CH1
	TIM2->CNT = 0;	// donde inicia el conteo
	TIM2->CR1 |= (1<<0);	// activar conteo

	//B. Tim21 para lcd, keypad y displays (1 ms)

	RCC->APB2ENR |= (1<<2);
	TIM21->PSC = 1600 - 1;
	TIM21->ARR = 10 - 1; //Para que sea cada 1 ms
	TIM21->CNT = 0;
	TIM21->DIER |= (1 << 0);   // que genere interrupcion en canal 1

	TIM21->CR1 |= (1<<0);	// activar conteo
	NVIC_EnableIRQ(TIM21_IRQn);   // Habilita interrupción global del TIM21

	// C. TIM22 clk interno (1s)
	RCC->APB2ENR |= (1 << 5);     // Habilita TIM22 clock
	TIM22->PSC = 1600 - 1;
	TIM22->ARR = 1000 - 1;
	TIM22->CNT = 0;
	TIM22->SR  &= ~(1 << 0);      // Limpia bandera de update
	TIM22->DIER |= (1 << 0);      // Habilita interrupción por update
	TIM22->CR1  |= (1 << 0);      // Arranca timer
	NVIC_EnableIRQ(TIM22_IRQn);

	// 5. Interrupciones

	//A. PC1 (Start)
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;   // Habilita SYSCFG (para mapear EXTI)

	// Mapear EXTI1 PC1
	SYSCFG->EXTICR[0] &= ~(0xF << 4);       // Limpia bits [7:4] para EXTI1
	SYSCFG->EXTICR[0] |=  (0x2 << 4);       // 0010 = Puerto C

	// Configurar EXTI1
	EXTI->IMR  |=  (1 << 1);                // Desbloquear interrupción línea 1
	EXTI->FTSR |=  (1 << 1);                // Flanco de bajada
	EXTI->RTSR &= ~(1 << 1);                // Sin flanco de subida

	// Habilitar interrupción global EXTI0_1 (porque cubre EXTI0 y EXTI1)
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	//B. PB11 (Cancelar)

	// Mapear EXTI11 PB11
	SYSCFG->EXTICR[2] &= ~(0xF << 12);
	SYSCFG->EXTICR[2] |=  (0x1 << 12);

	// Configurar EXTI11
	EXTI->IMR  |=  (1 << 11);
	EXTI->FTSR |=  (1 << 11);
	EXTI->RTSR &= ~(1 << 11);

	//C. PB12


	// Mapear EXTI11 PB12
	SYSCFG->EXTICR[3] &= ~(0xF << 0);
	SYSCFG->EXTICR[3] |=  (0x1 << 0);

	// Configurar EXTI12
	EXTI->IMR  |=  (1 << 12);
	EXTI->FTSR |=  (1 << 12);
	EXTI->RTSR |= (1 << 12);




	// Habilitar interrupción global EXTI4_15 (maneja líneas 4–15)
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	//6. USART2
	RCC->APB1ENR |= (1 << 17);      // Habilitar reloj USART2

	// PA2 (TX) y PA3 (RX) → modo alternativo AF4
	GPIOA->MODER &= ~((3 << (2*2)) | (3 << (3*2)));
	GPIOA->MODER |=  ((2 << (2*2)) | (2 << (3*2)));  // AF
	GPIOA->AFR[0] &= ~((0xF << (2*4)) | (0xF << (3*4)));
	GPIOA->AFR[0] |=  ((4 << (2*4)) | (4 << (3*4))); // AF4 = USART2

	// Configuración USART2 (115200 8N1)
	USART2->BRR = 139;              // 16MHz / 115200 ≈ 138.9
	USART2->CR1 = (1 << 3) | (1 << 2); // TE y RE habilitados
	USART2->CR1 |= (1 << 0);        // UE: USART habilitado


	//7. Reinicio de todo
	GPIOB->ODR &= ~((1 << 8) | (1 << 9)); //Apago motor



}

// <-------FUNCIONES DEL MOTOR-------->

void lavado(void)
{
	GPIOA->ODR |= (1<<15);
	GPIOA->ODR &= ~(1<<12);
	GPIOB->ODR &= ~(1<<10);

    TIM2->CCR1 = 23;
    GPIOC->ODR |=  (1 << 2);   // IN1 = 1
    GPIOC->ODR &= ~(1 << 3);   // IN2 = 0 → giro horario
    sentido = 0;               // sentido fijo

}

void enjuague(void)
{
    static uint32_t last_toggle = 0;
    static uint8_t sentido_local = 0;
    static uint8_t pwm_actual = 20; // duty dinámico

    // ----- LED control -----
    GPIOA->ODR &= ~(1 << 15);  // apaga LED lavado
    GPIOA->ODR |=  (1 << 12);  // enjuague ON
    GPIOB->ODR &= ~(1 << 10);  // apaga centrifugado

    // ----- Velocidad base -----
    if (pwm_actual < 28) pwm_actual++;  // pequeña rampa
    TIM2->CCR1 = pwm_actual;            // ~55% duty

    // ----- Cambio de sentido cada 200 ms -----
    if ((tick_ms - last_toggle) >= 200)
    {
        last_toggle = tick_ms;
        sentido_local ^= 1;

        if (sentido_local == 0) {
            GPIOC->ODR |=  (1 << 2);   // CW
            GPIOC->ODR &= ~(1 << 3);
        } else {
            GPIOC->ODR &= ~(1 << 2);   // CCW
            GPIOC->ODR |=  (1 << 3);
        }
    }
}

void centrifugado(void)
{
	GPIOA->ODR &= ~(1<<12);
	GPIOA->ODR &= ~(1<<15);
	GPIOB->ODR |= (1<<10);

    TIM2->CCR1 = 25;
    GPIOC->ODR |=  (1 << 2);   // IN1 = 1
    GPIOC->ODR &= ~(1 << 3);   // IN2 = 0 → sentido horario
    sentido = 0;
}

//<-------Funciones LCD-------->
void LCD_SendNibble(uint8_t nibble) {
    uint32_t mask = (1<<8)|(1<<9)|(1<<10)|(1<<11);
    GPIOA->BSRR = (mask<<16);
    GPIOA->BSRR = ((nibble & 0x01)?(1<<8):0)
                | ((nibble & 0x02)?(1<<9):0)
                | ((nibble & 0x04)?(1<<10):0)
                | ((nibble & 0x08)?(1<<11):0);

    GPIOA->BSRR = (1<<5); // E=1
    for (volatile int i=0;i<10;i++); // corto
    GPIOA->BSRR = (1<<(5+16)); // E=0
}

void LCD_SendByte(uint8_t data, uint8_t rs) {
    if (rs) GPIOA->BSRR = (1<<4); else GPIOA->BSRR = (1<<(4+16));
    LCD_SendNibble((data >> 4) & 0x0F);
    LCD_SendNibble(data & 0x0F);
}

void LCD_Service_1ms(void) {
    static uint16_t wait = 0;

    if (wait > 0) { wait--; return; }

    switch (lcd_state) {
        case 0: // Inicialización paso a paso
            LCD_SendNibble(0x03); wait = 5; lcd_step++;
            if (lcd_step >= 3) { lcd_step = 0; lcd_state = 1; }
            break;

        case 1:
            LCD_SendNibble(0x02); wait = 5; lcd_state = 2;
            break;

        case 2:
            LCD_SendByte(0x28, 0); wait = 2; lcd_state = 3;
            break;

        case 3:
            LCD_SendByte(0x0C, 0); wait = 2; lcd_state = 4;
            break;

        case 4:
            LCD_SendByte(0x06, 0); wait = 2; lcd_state = 5;
            break;

        case 5:
            LCD_SendByte(0x01, 0); wait = 3; lcd_state = 6;
            break;

        case 6:
            // listo para escribir
            LCD_SendByte(lcd_text[lcd_index], 1);
            lcd_index++;
            if (lcd_text[lcd_index] == '\0') lcd_state = 7;
            wait = 2;
            break;

        case 7:
            // LCD inactiva
            break;

        case 8: // nuevo estado: limpiar antes de imprimir
            LCD_SendByte(0x01, 0);   // clear
            wait = 3;                // 3ms aprox
            lcd_state = 9;           // paso siguiente
            break;
        case 9: // ahora sí, escribir texto
            LCD_SendByte(lcd_text[lcd_index], 1);
            lcd_index++;
            if (lcd_text[lcd_index] == '\0') lcd_state = 7;
            wait = 2;
            break;
    }
}

void LCD_PrintNew(const char *texto) {
    lcd_text = texto;      // guardamos puntero al texto
    lcd_index = 0;         // reiniciamos contador de caracteres
    lcd_state = 8;         // nuevo estado: limpiar antes de escribir
}

//<-------Funciones Keypad-------->
void tecla_activa(void)
{
    static uint8_t tecla_anterior = 0;
    static uint16_t contador_estable = 0;

    uint8_t tecla_actual = 0;

    // --- Escaneo rápido ---
    GPIOB->ODR |= (1<<7)|(1<<8)|(1<<9);

    GPIOB->ODR &= ~(1<<7);
    for (volatile int i = 0; i < 300; i++) __NOP();
    if (!(GPIOC->IDR & (1<<4))) tecla_actual = 1;
    GPIOB->ODR |= (1<<7);

    GPIOB->ODR &= ~(1<<8);
    for (volatile int i = 0; i < 300; i++) __NOP();
    if (!(GPIOC->IDR & (1<<4))) tecla_actual = 2;
    GPIOB->ODR |= (1<<8);

    GPIOB->ODR &= ~(1<<9);
    for (volatile int i = 0; i < 300; i++) __NOP();
    if (!(GPIOC->IDR & (1<<4))) tecla_actual = 3;
    GPIOB->ODR |= (1<<9);

    // --- Lógica de debounce ---
    if (tecla_actual == tecla_anterior && tecla_actual != 0)
    {
        contador_estable++;
        if (contador_estable >= 3)   // 3 lecturas estables de 20 ms = 60 ms
        {
            if (tecla_actual != ciclo)  // si es nueva tecla
                ciclo = tecla_actual;

            contador_estable = 0; // reinicia para evitar múltiples lecturas
        }
    }
    else
    {
        contador_estable = 0;
    }

    tecla_anterior = tecla_actual;
}

//<-----Funciones Display----->
uint8_t parser(uint8_t decode){
    switch(decode){
        case 0: return cc_0;
        case 1: return cc_1;
        case 2: return cc_2;
        case 3: return cc_3;
        case 4: return cc_4;
        case 5: return cc_5;
        case 6: return cc_6;
        case 7: return cc_7;
        case 8: return cc_8;
        case 9: return cc_9;
        default: return cc_all_off;
    }
}

void print_display(void)
{
    uint32_t temp;

    switch (en_control)
    {
        case 0:
            // 🔹 Activar D0 (izquierda)
            GPIOC->BSRR = (D3_ctrl << 16); // Apagar D3
            GPIOC->BSRR = D0_ctrl;         // Encender D0

            temp = GPIOB->ODR;                     // Leer ODR
            temp &= ~(0x7F);                       // Limpiar PB0–PB6
            temp |= parser(display_clk[0]) & 0x7F; // Cargar segmentos
            GPIOB->ODR = temp;                     // Escribir
            en_control++;
            break;

        case 1:
            // 🔹 Activar D1
            GPIOC->BSRR = (D0_ctrl << 16);
            GPIOC->BSRR = D1_ctrl;

            temp = GPIOB->ODR;
            temp &= ~(0x7F);
            temp |= parser(display_clk[1]) & 0x7F;
            GPIOB->ODR = temp;
            en_control++;
            break;

        case 2:
            // 🔹 Activar D2
            GPIOC->BSRR = (D1_ctrl << 16);
            GPIOC->BSRR = D2_ctrl;

            temp = GPIOB->ODR;
            temp &= ~(0x7F);
            temp |= parser(display_clk[2]) & 0x7F;
            GPIOB->ODR = temp;
            en_control++;
            break;

        case 3:
            // 🔹 Activar D3 (derecha)
            GPIOC->BSRR = (D2_ctrl << 16);
            GPIOC->BSRR = D3_ctrl;

            //Esto me permite usar ODR guardando los datos de PB7 hacia arriba y solo modificar PB0-PB6
            temp = GPIOB->ODR;
            temp &= ~(0x7F);
            temp |= parser(display_clk[3]) & 0x7F;
            GPIOB->ODR = temp;

            en_control = 0;
            break;

        default:
            en_control = 0;
            break;
    }
}

//<------Funciones conteo regresivo------->
void actualizar_display_desde_segundos(void)
{
    uint8_t minutos = tiempo_total / 60;
    uint8_t segundos = tiempo_total % 60;

    display_clk[0] = minutos / 10;  // decenas de minuto
    display_clk[1] = minutos % 10;  // unidades de minuto
    display_clk[2] = segundos / 10; // decenas de segundo
    display_clk[3] = segundos % 10; // unidades de segundo
}

void clk_inverso(void)
{
    if (tiempo_total > 0)
    {
        tiempo_total--;                   // 🔹 resta 1 segundo
        actualizar_display_desde_segundos(); // 🔹 actualiza los 4 dígitos
    }
    else
    {
        ciclo_activo = 0;

        //Motor desactivado
        GPIOC->ODR &=  ~(1 << 2);   // IN1 = 1
        GPIOC->ODR &= ~(1 << 3);   // IN2 = 0 → giro horario

        //Leds apagadas
        GPIOB->ODR &= ~(1 << 10);

        //
        USART2_write_string("Ciclo terminado\r\n");
        ciclo = 0;
    }
}

//<-----Funciones USART2----->
void USART2_write_char(char ch) {
    while (!(USART2->ISR & (1 << 7))); // Esperar TXE
    USART2->TDR = ch;
}

void USART2_write_string(const char *str) {
    while (*str) {
        USART2_write_char(*str++);
    }
}

void USART2_write_uint(uint32_t num) {
    char buffer[12];
    int i = 0;

    if (num == 0) {
        USART2_write_char('0');
        return;
    }

    while (num > 0 && i < 10) {
        buffer[i++] = '0' + (num % 10);
        num /= 10;
    }
    while (i--) USART2_write_char(buffer[i]);
}

//<------Funciones Logica Lavadora------->
void selec_ciclo(){

	if(ciclo != ciclo_prev && ciclo_activo != 1){
		ciclo_prev = ciclo;

		switch(ciclo){
		    case 1:
		        LCD_PrintNew("Ciclo Rapido 3M");
		        USART2_write_string("Ciclo Rapido 3M seleccionado\r\n");
		        tiempo_total = 180; // 3 minutos

		        display_clk[0] = 0;  // decenas de minuto
		        display_clk[1] = 3;  // unidades de minuto
		        display_clk[2] = 0; // decenas de segundo
		        display_clk[3] = 0; // unidades de segundo


		        break;
		    case 2:
		        LCD_PrintNew("Ciclo Normal 6M");
		        USART2_write_string("Ciclo Normal 6M seleccionado\r\n");
		        tiempo_total = 360; // 6 minutos
		        display_clk[0] = 0;  // decenas de minuto
		        display_clk[1] = 6;  // unidades de minuto
		        display_clk[2] = 0; // decenas de segundo
		        display_clk[3] = 0; // unidades de segundo
		        break;
		    case 3:
		        LCD_PrintNew("Ciclo Pesado 9M");
		        USART2_write_string("Ciclo Pesado 9M seleccionado\r\n");
		        tiempo_total = 540; // 9 minutos
		        display_clk[0] = 0;  // decenas de minuto
		        display_clk[1] = 9;  // unidades de minuto
		        display_clk[2] = 0; // decenas de segundo
		        display_clk[3] = 0; // unidades de segundo
		        break;
		}

	}

}

void control_etapa(){
	switch(etapa){
	case 1:
		lavado();
		break;

	case 2:
		enjuague();
		break;
	case 3:
		centrifugado();
		break;

	}

}

//<-------Handlers de interrupciones-------->
void TIM21_IRQHandler(void)
{
	static uint8_t toggle = 0;
    if (TIM21->SR & (1<<0)) // Canal 1
    {
        TIM21->SR &= ~(1<<0);  // limpiar flag
        tick_ms++;

        LCD_Service_1ms();     // cada 1 ms

        // cada 2 ms
        if (tick_ms % 2 == 0){
        	print_display();
        }

        // cada 20 ms: escanear keypad
        if (tick_ms % 20 == 0){
        	 tecla_activa();
        	 selec_ciclo();
        }


        //Activar, desactivar buzzer
        if (buzzer_ms > 0) {
            toggle ^= 1; // alterna 0-1 cada ms
            if (toggle)
                GPIOA->ODR |=  (1 << 6);  // ON
            else
                GPIOA->ODR &= ~(1 << 6);  // OFF

            buzzer_ms--;
        } else {
            GPIOA->ODR &= ~(1 << 6); // asegurarse que quede apagado
        }

        // Mostrar "Seleccione ciclo" después de 2 segundos del fin
        if (fin_ciclo_tick > 0 && tick_ms >= fin_ciclo_tick) {
            fin_ciclo_tick = 0;
            if (ciclo_activo == 0 && ciclo == 0) {
                LCD_PrintNew("Seleccione ciclo");
            }
        }


    }
}

void TIM22_IRQHandler(void)
{
    if (TIM22->SR & (1 << 0)) // Canal 1
    {
        TIM22->SR &= ~(1 << 0);
        tick_seg++;

        if (ciclo_activo)
        {
            clk_inverso();        // Actualiza el display regresivo
            segundos_etapa++;     // Contador interno por etapa
            control_etapa();      // Ejecuta comportamiento según etapa

            // 🔹 Duración por etapa según el ciclo
            uint16_t etapa_duracion = 0;
            switch (ciclo)
            {
                case 1: etapa_duracion = 60;  break;  // 1 min
                case 2: etapa_duracion = 120; break;  // 2 min
                case 3: etapa_duracion = 180; break;  // 3 min
                default: etapa_duracion = 60; break;  // seguridad
            }

            // 🔸 CAMBIO AUTOMÁTICO DE ETAPAS
            if (etapa == 1 && segundos_etapa >= etapa_duracion) {
                etapa = 2;
                segundos_etapa = 0;
                USART2_write_string("Cambio -> Enjuague\r\n");
        	    LCD_PrintNew("Enjuague");
        	    USART2_write_string("Etapa Enjuague\r\n");
            }
            else if (etapa == 2 && segundos_etapa >= etapa_duracion) {
                etapa = 3;
                segundos_etapa = 0;
                USART2_write_string("Cambio -> Centrifugado\r\n");
        	    LCD_PrintNew("Centrifugado");
        	    USART2_write_string("Etapa: Centrifugado\r\n");
            }
            else if (etapa == 3 && tiempo_total == 0) {
                segundos_etapa = 0;
                etapa = 0;
                ciclo_activo = 0;
                ciclo = 0;

                LCD_PrintNew("Ciclo Finalizado");
                USART2_write_string("Ciclo completo!\r\n");

                //Motor desactivado
                GPIOC->ODR &= ~((1 << 2) | (1 << 3));

                //Leds apagadas
                GPIOA->ODR &= ~((1 << 12) | (1 << 15));
                GPIOB->ODR &= ~(1 << 10);

                // 🔊 Buzzer no bloqueante: 1 segundo
                buzzer_ms = 1000;

                fin_ciclo_tick = tick_ms + 2000;
            }
        }
    }
}

void EXTI0_1_IRQHandler(void)
{
	if (EXTI->PR & (1 << 1)) {
	    EXTI->PR = (1 << 1);

	    if (ciclo != 0) {
	        USART2_write_string("Boton START presionado\r\n");
		    LCD_PrintNew("Lavado");
		    USART2_write_string("Etapa Lavado\r\n");
	        ciclo_activo = 1;\
	        etapa = 1;
	    } else {
	        USART2_write_string("No hay ciclo seleccionado\r\n");
	        LCD_PrintNew("No ciclo, elija");
	        ciclo_activo = 0;
	    }
	}
}

void EXTI4_15_IRQHandler(void){
	if (EXTI->PR & (1 << 11)) {
		    EXTI->PR = (1 << 11);

		    if (ciclo == 0) {
		    	LCD_PrintNew("Sin ciclo activo");
		        USART2_write_string("No hay ciclo activo para cancelar\r\n");
		    } else {
		    	LCD_PrintNew("Seleccione Ciclo");
		        USART2_write_string("Ciclo CANCELADO\r\n");

		        //Borramos ciclo y tiempo
		        ciclo_activo = 0;
		        ciclo = 0;
		        tiempo_total = 0;
		        segundos_etapa = 0;

		        //Apagamos motor
		        GPIOC->ODR &=  ~(1 << 2);   // IN1 = 0
		        GPIOC->ODR &= ~(1 << 3);   // IN2 = 0

		        //Apagar Led
		        GPIOA->ODR &= ~(1<<12);
		        GPIOA->ODR &= ~(1<<15);
		        GPIOB->ODR &= ~(1<<10);

		        //Reiniciamos 00:00
		        actualizar_display_desde_segundos();
		    }
		}

	if (EXTI->PR & (1 << 12)) {
	    EXTI->PR = (1 << 12); // limpiar flag

	    // Leemos estado actual de la tapa
	    uint8_t tapa_cerrada = (GPIOB->IDR & (1 << 12)) ? 1 : 0;

	    if (!tapa_cerrada) {
	        USART2_write_string("Cierre tapa \r\n");
	        LCD_PrintNew("Tapa abierta!");
	        ciclo_activo = 0;

	        // Apagar motor y LEDs
	        GPIOC->ODR &= ~((1 << 2) | (1 << 3));
	        GPIOA->ODR &= ~((1 << 12) | (1 << 15));
	        GPIOB->ODR &= ~(1 << 10);

	        // Aviso sonoro corto
	        buzzer_ms = 300;
	    }
	    else {
	    	USART2_write_string("Reanudando..");
	        buzzer_ms = 100;

	        // Solo reanudar si había un ciclo activo previamente
	        if (tiempo_total > 0 && ciclo != 0 && etapa != 0) {
	            ciclo_activo = 1;
	        }
	    }
	}

}

int main(){
system_init();
	while(1){

	}
}
