# Serie 2 – Implementación de FSM con Multiplexación de Displays y Programación en FPGA (Basys3)

---

## 🧠 Descripción General

Este proyecto implementa y complementa el sistema **VivePass**, una máquina de estados finitos (**FSM**) ejecutada en **hardware real (FPGA Basys3)**.  
El sistema combina la lógica secuencial de control vehicular (garita de acceso) con una **interfaz visual multiplexada** que muestra tanto el estado del sistema como un **reloj digital 24 h**, todo dentro del mismo hardware.

El diseño está escrito en **SystemVerilog**, sintetizado en **Vivado**, y mapeado a los recursos físicos de la **Basys3 (Artix-7)**.

---

## ⚙️ Módulos Funcionales Principales

### 🧩 Sistema VivePass (FSM)
Implementa dos máquinas de estados finitos:

- **M1 – FSM Moore (lector de código)**  
  Determina la longitud (`L`) del código/sticker leído (0–3 bits).  
  Se reinicia con la señal de confirmación (`C`) proveniente de la Mealy.

- **M2 – FSM Mealy (control de talanquera)**  
  Recibe la longitud (`L`) y el estado del sensor de vehículo (`ST`).  
  Si `L == 3` y `ST == 1`, activa la apertura (`A = 1`) y envía confirmación (`C = 1`) a la Moore.

---

### ⏱️ Reloj Digital (Clock 24 h)
Genera y actualiza la hora local en formato **HH:MM**, mostrada en los **displays de 7 segmentos** de la Basys3.  
Se implementa mediante **contadores síncronos** que simulan segundos, minutos y horas, con una base de tiempo ajustable según la frecuencia del reloj principal (100 MHz).

---

## 💡 Multiplexación de Displays

El sistema utiliza los **cuatro displays integrados** de la Basys3 mediante **multiplexación temporal**, activando cada display por turnos cada ~1 ms.  
Esto genera la ilusión de que todos están encendidos simultáneamente.

### Vistas disponibles:
- **Vista VivePass (FSM):**
  - Display 0 → Conteo de dígitos (`dread`)
  - Displays restantes apagados.

- **Vista Reloj (Clock):**
  - Displays [3:2] → Horas (HH)
  - Displays [1:0] → Minutos (MM)

El cambio de vista se realiza con el **botón U**, el cual pasa por módulos de **debounce** y **one_pulse** para evitar rebotes y señales duplicadas.

---

## 🔩 Arquitectura del Sistema

### Módulos Principales

| Módulo | Descripción |
|---------|--------------|
| **FSM_Moore** | Determina la longitud del código (0–3). Controla la vista del sistema (FSM o reloj). |
| **FSM_Mealy** | Controla la apertura/cierre de la barrera y confirma lecturas válidas. |
| **clock** | Implementa el reloj 24 h con contadores de segundos, minutos y horas. |
| **display_7segments** | Multiplexa los 4 displays, decodifica números BCD y maneja ánodos comunes activos en bajo. |
| **clk_psc** | Divisor de reloj para tareas no críticas (ej. visualización lenta). |
| **debouncer / one_pulse** | Limpian las señales de botones para evitar rebotes. |
| **Top_basys3** | Módulo superior: integra todo, define la multiplexación entre vistas y mapea los puertos físicos. |

---

## ⚙️ Entradas y Salidas (Basys3)

| Señal | Tipo | Descripción |
|-------|------|--------------|
| `clk` | Entrada | Reloj de 100 MHz de la Basys3 |
| `reset` | Entrada | Reinicio global del sistema |
| `btnU` | Entrada | Alterna entre vista FSM y reloj |
| `D` | Entrada | Bit leído del sticker (0/1) |
| `ST` | Entrada | Sensor del vehículo |
| `A` | Salida | Control de talanquera (1 = abrir) |
| `C` | Salida | Confirmación de lectura válida |
| `an[3:0]` | Salida | Activación de displays (ánodo común, activos en bajo) |
| `seg[6:0]` | Salida | Segmentos del display (activos en bajo) |
| `dp` | Salida | Punto decimal (apagado) |

---

## 🧭 Flujo del Sistema

1. El vehículo llega y el sensor (`ST`) se activa.  
2. El lector envía bits (`D`) a la **FSM Moore**, que cuenta la longitud.  
3. La **FSM Mealy** valida si `L == 3` y abre la talanquera (`A = 1`).  
4. La **Moore** recibe la confirmación (`C = 1`) y reinicia la lectura.  
5. El usuario puede alternar entre **vista de sistema** y **reloj local** con `btnU`.

---

## ⚙️ Multiplexación de los Displays

- **1 ms por display** (~1 kHz de refresco).  
- **Activos en bajo:** `an` y `seg`.  
- **Controlados por contador de 17 bits** que alterna cada flanco positivo de `clk`.

---

## 🎥 Evidencias en Video

### Ejercicio 1 – Implementación de FSM
- FSM con reloj integrado:
  👉 [https://youtu.be/OaL7UdaPSfI](https://youtu.be/CA0SrCMLTzY?si=9Zm16J9bX2bOYJK3)  
- Explicación del código:  
  👉 [https://youtu.be/86AQyynaU6M](https://youtu.be/1jGhOcW8-Ik)

### Ejercicio 2 – Flujo de Vivado
Demostración del flujo completo de diseño en Vivado:
1. **RTL Schematic (Elaborated Design)** – Vista lógica antes de síntesis.  
2. **Synthesis Design** – Traducción del RTL a celdas lógicas.  
3. **Implementation Design** – Colocación y ruteo físico (LUTs, DRC, Slack).  
4. **Bitstream / Hardware Manager** – Generación del `.bit` y programación en FPGA.

📺 Explicación completa:  
👉 [https://youtu.be/46htZkBVLcM]()
👉 [https://youtu.be/46htZkBVLcM](https://youtu.be/XVCBWMOdC5Q)






