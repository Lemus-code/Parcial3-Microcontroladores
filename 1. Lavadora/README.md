# Serie 3 – Sistema de Lavadora Inteligente con Control por FSM y Multiplexación de Interfaces (STM32L053R8)

---

## 🧠 Descripción General

Este proyecto implementa una **lavadora automática embebida** basada en una **máquina de estados finitos (FSM)**, ejecutada completamente en **hardware real (STM32 Nucleo-L053R8)**.  

El sistema coordina el funcionamiento del **motor**, **display**, **teclado**, **LCD**, **buzzer** y **sensores**, integrando control de etapas, cuenta regresiva, multiplexación de displays y seguridad por tapa bajo una arquitectura modular programada en **C bare-metal**.

---

## ⚙️ Módulos Funcionales Principales

### 🧩 1. FSM de Control del Ciclo de Lavado
Define la secuencia **Lavado → Enjuague → Centrifugado**, gestionando tiempos, direcciones del motor y alertas visuales.

| Ciclo | Duración | Descripción |
|--------|-----------|-------------|
| Rápido | 3 min | Lavado ligero con menos tiempo de rotación |
| Normal | 6 min | Lavado estándar con alternancia de sentidos |
| Pesado | 9 min | Mayor duración y fuerza de centrifugado |

---

### ⚙️ 2. Control del Motor (Driver L298N)
El motor del tambor es controlado mediante **PWM (TIM2_CH1 – PA0)** para variar la velocidad, y **líneas IN1/IN2 (PC2–PC3)** para definir el sentido de giro.

- **Lavado:** sentido horario constante.  
- **Enjuague:** alternancia CW/CCW cada 500 ms con rampa de duty.  
- **Centrifugado:** duty alto (≈70 %) con sentido fijo horario.

---

### 🔢 3. Display de 7 Segmentos Multiplexado
Los cuatro dígitos son controlados por multiplexación temporal (~1 ms por dígito).  
Refrescados por **TIM21**, muestran la cuenta regresiva o el tiempo de espera.

| Dígito | Información |
|---------|--------------|
| D1 | Decenas de minuto |
| D2 | Unidades de minuto |
| D3 | Decenas de segundo |
| D4 | Unidades de segundo |

---

### 🧭 4. LCD 16×2 (Interfaz Usuario)
Controlado en modo 4 bits (PA4–PA11) con su propia FSM.  
Mensajes dinámicos:

- `Select ciclo:123`  
- `Ciclo Pesado 9M seleccionado`  
- `Esperando inicio...`  
- `Tapa abierta!`  
- `Ciclo Finalizado`

---

### ⌨️ 5. Keypad (Selección y Configuración)
Matriz 2×4 escaneada por software cada 20 ms:
- **1–3:** seleccionan ciclo.  
- **A:** guarda configuración.  
- **B:** cancela.  

En **modo configuración**, permite sumar tiempo de espera (+1 min, +5 min, +10 min, +30 min, +1 h).

Incluye lógica de **debounce** y **auto-repetición**.

---

### 🔔 6. Buzzer y LEDs
- **PA6 (PWM):** buzzer para avisos.  
- **PA12, PA15, PB10:** LEDs de etapa  
  - Lavado / Enjuague / Centrifugado.

---

### 🚨 7. Seguridad por Tapa
Switch en **PB12 (EXTI12)**:
- **Abierta:** pausa ciclo, apaga motor y activa buzzer.  
- **Cerrada:** reanuda o inicia según estado.  
Compatible con modo “espera programada”.

---

### 💬 8. Comunicación Serial (USART2)
Interfaz de diagnóstico a 115 200 bps.

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



