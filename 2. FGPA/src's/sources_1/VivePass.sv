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
//        Fila = PC2  → entrada con pull-up interno
//        Columnas = PB7–PB9 → salidas controladas por ODR
//        → C1 = 30 min, C2 = 40 min, C3 = 50 min

// Motor del tambor (controlado por driver L298N):
//        IN1 = PC3     → Dirección 1 (sentido horario)
//        IN2 = PC4     → Dirección 2 (sentido antihorario)
//        ENA (PWM velocidad) = PA6 (TIM3_CH1)
//        → Control de velocidad mediante PWM (Timer 3 Canal 1)
//        → Inversión de rotación controlada por software usando IN1/IN2

// Buzzer (alertas de inicio, fin, error):
//        PWM salida = PA6 (TIM2_CH1)
//        → Generación de tonos con Timer 2 Canal 1

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
