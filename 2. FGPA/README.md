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

## 🧩 Arquitectura del Sistema

