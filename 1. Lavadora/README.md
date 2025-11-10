
---

## 🔩 Arquitectura del Sistema

| Módulo | Descripción |
|---------|--------------|
| `system_init()` | Inicializa relojes, GPIOs, timers, interrupciones y USART. |
| `lavado()`, `enjuague()`, `centrifugado()` | Rutinas de etapa. |
| `LCD_Service_1ms()` | FSM de LCD, ejecutada por interrupción de 1 ms. |
| `print_display()` | Multiplexación de displays. |
| `tecla_activa()` | Escaneo del keypad con debounce. |
| `clk_inverso()` / `clk_inverso_espera()` | Cuentas regresivas. |
| `TIM21_IRQHandler()` | LCD, display, buzzer, teclado. |
| `TIM22_IRQHandler()` | FSM principal (etapas y tiempo). |
| `EXTI0_1 / EXTI4_15_IRQHandler()` | START, CANCEL, TAPA. |

---

## 🧭 Flujo General

1. Selección de ciclo con teclas **1–3**.  
2. Configuración opcional de espera (tecla **A**).  
3. START inicia ciclo o modo espera según configuración.  
4. Secuencia automática de etapas:
   - Lavado → Enjuague → Centrifugado.  
5. Fin de ciclo: buzzer, LEDs off, LCD muestra *“Ciclo Finalizado”*.  
6. Seguridad activa si la tapa se abre.

---

## ⚙️ Hardware Utilizado

| Componente | Conexión |
|-------------|-----------|
| STM32 Nucleo-L053R8 | MCU principal |
| L298N | Driver del motor |
| LCD 16×2 (modo 4 bits) | PA4–PA11 |
| Display 7 segmentos×4 | PB0–PB6, PC5/6/8/9 |
| Keypad 2×4 | Filas: PC0/PC4 · Columnas: PB7–PB9, PA1 |
| Buzzer PWM | PA6 |
| LEDs etapas | PA12, PA15, PB10 |
| Switch tapa | PB12 (EXTI12) |
| Botones START/CANCEL | PC1 (EXTI1), PB11 (EXTI11) |
| USART2 | PA2 TX, PA3 RX |

---

## ⏱️ Timers e Interrupciones

| Timer | Frecuencia | Función |
|--------|-------------|----------|
| TIM2 | 20 kHz PWM | Motor |
| TIM21 | 1 kHz | LCD, display, teclado, buzzer |
| TIM22 | 1 Hz | FSM principal |

| EXTI | Evento | Acción |
|------|---------|--------|
| PC1 | START | Inicia ciclo |
| PB11 | CANCEL | Cancela ciclo/config |
| PB12 | Tapa | Pausa o reanuda |


---

## ⭐ Características Destacadas

- FSM multinivel: **Ciclo → Etapa → Evento**.  
- Multiplexación no bloqueante de LCD y display.  
- PWM variable en motor y buzzer.  
- Modo configuración con temporizador programable.  
- Seguridad física por sensor de tapa.  
- Comunicación serial para depuración.  
- Modularidad total en C (bare-metal STM32).

---

## 🎥 Pruebas Sugeridas

1. Selección y ejecución de cada ciclo.  
2. Verificación de cambio de etapas.  
3. Modo espera programada (tecla A + START).  
4. Apertura y cierre de tapa en ejecución.  
5. Cancelación manual (tecla B).  
6. Monitoreo vía serial a 115 200 bps.

---


