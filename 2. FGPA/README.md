# VivePass + Reloj Local (Basys3 · SystemVerilog)

> Sistema de acceso vehicular basado en **FSM Moore/Mealy** con visualización en **7 segmentos** para el conteo de dígitos leídos y un **reloj local (hh:mm)** en la tarjeta Basys3.

---

## 🎯 Objetivo

- **VivePass** sustituye el uso de tickets por un **sticker único** en el vehículo.  
- El sistema está compuesto por:
  - **FSM Moore:** mide la **longitud** del código leído (`L = 0..3`).
  - **FSM Mealy:** controla la **talanquera**, abriéndola (`A = 1`) solo si `ST = 1` y `L == 3`, confirmando lectura con `C`.

Esta versión añade:
- Un **display de 7 segmentos** que muestra el **conteo de dígitos** o el **reloj local**.
- Un **reloj hh:mm (24h)** completamente funcional y sincronizado con el `clk` de la Basys3.

---


---

## 🔌 Basys3 (pines y polaridades)

- **Reloj:** `clk` @ W5 (100 MHz)  
- **7-segmentos (ánodo común, activos en bajo)**  
  - `an[3:0]`: U2, U4, V4, W4  
  - `seg[6:0]`: W7, W6, U8, V8, U5, V5, U7  
  - `dp`: V7 (mantener en ‘1’ si no se usa)
- **Botones:** `reset` = U18, `btnU` = T18  
- **IOSTANDARD:** LVCMOS33  

> El módulo `display_7segments` ya entrega señales activas en bajo para `an` y `seg`.

---

## ⚙️ Funcionamiento

- **FSM Moore**
  - `D && ~C` → incrementa `dread` (máx. 3) y avanza S0→S3.  
  - `~D && C` → limpia el conteo y vuelve a S0.  
  - `L` se asigna según el estado (00..11).

- **FSM Mealy**
  - `A = 1` solo si `ST = 1` y `L = 3`.  
  - `C = 1` confirma lectura **válida** (`ST = 1` y `L = 3`).

- **Reloj (hh:mm)**
  - 60 s → +1 min; 60 min → +1 h; 24 h → 00:00.  
  - Ajustar los contadores según la frecuencia real de `clk`.

- **Display**
  - `select = 0` → `d0 = dread`, `d1–d3 = 0` (modo VivePass)  
  - `select = 1` → `d0 = min_u`, `d1 = min_d`, `d2 = hora_u`, `d3 = hora_d` (modo reloj)  
  - Multiplexado ≈ 1 ms/dígito.

---

## 🛠️ Compilación (Vivado)

1. Crear proyecto **Basys3 · Artix-7**.  
2. Agregar fuentes desde `src/` y restricciones de `constraints/basys3.xdc`.  
3. Verificar coincidencia de nombres (`clk`, `reset`, `an`, `seg`).  
4. Usar un **solo reloj principal (`clk`)** para todas las FSM.  
5. Sintetizar, implementar y programar.

---

## ⏱️ Timing

| Función   | Frecuencia | Ciclos @ 100 MHz |
|------------|-------------|------------------|
| Display (~1 ms) | 1 kHz | 100 000 |
| Reloj (1 s)     | 1 Hz  | 100 000 000 |

> Si usas `clk_psc.sv` como divisor, recalcula los umbrales según la nueva frecuencia.

---


