# Electrical Wiring & Signal Architecture

This document illustrates the power distribution, motor drive, and dual-encoder signal routing for the **STM32 Nucleo-F446RE** Furuta pendulum hardware platform.

---

## System Wiring Diagram

```text
+-----------------------+       +---------------------------+       +---------------------+
|   Power Supply        |       |   Motor Driver Module     |       |  12 V DC Geared     |
|   12 V / 3 A DC       |------>|   (TB6612FNG / H-Bridge)  |======>|  Motor (9.267:1)    |
|                       |  GND  |                           | Power |                     |
+-----------+-----------+  +--->| VM (Motor Supply)         |       +----------+----------+
            |              |    | GND (Common Ground)       |                  |
            |              |    |                           |                  | Mechanical
            v              |    | PWM  <-- TIM8_CH2         |                  | Coupling
+-----------+-----------+  |    | IN1  <-- PA10             |                  v
|   Buck Converter      |  |    | IN2  <-- PB3              |       +---------------------+
|   (MP1584EN: 5V/3.3V) |  |    +---------------------------+       |  Rotary Arm         |
+-----------+-----------+  |                                        +----------+----------+
            |              |                                                   |
            v              |                                                   v
+-----------+-----------+  |    +---------------------------+       +---------------------+
|   STM32 Nucleo-F446RE |  |    |  Motor Shaft Encoder      |       |  Conductive         |
|                       |  |    |  (44 counts/rev)          |<------|  Slip Ring          |
|  TIM1 (4x Quad)       |<-+----|  Channel A / B            |       |  (6-ch, 5 mm ID)    |
|                       |       +---------------------------+       +----------+----------+
|                       |                                                      |
|  TIM3 (4x Quad)       |<------[Level Shifter]<--[Slip Ring]                  |
|                       |       +---------------------------+                  |
|  USART2 (115200 bps)  |       |  Pendulum Pivot Encoder   |<-----------------+
|  PC13 (Blue Button)   |       |  (4096 counts/rev)        |
+-----------+-----------+       +---------------------------+       +---------------------+
            |                                                       |  Pendulum Rod       |
            +-----------------> Common GND (Mandatory)              +---------------------+
```

---

## Key Integration Notes

1. **Common Ground**: The 12 V motor power ground, buck converter ground, encoder ground, and Nucleo-F446RE `GND` must share a common reference plane.
2. **Continuous Rotation via Slip Ring**: Pendulum encoder power and A/B quadrature channels pass through the hollow-shaft conductive slip ring so the rotary arm can spin continuously without twisting cables.
3. **Voltage Level Translation**: 5 V quadrature encoder outputs are shifted down to 3.3 V before entering STM32 `TIM3` / `TIM1` timer inputs.
4. **Software Safety Protections**: Firmware enforces `PWM_DUTY_SAFETY_CAP = 0.80` (`TIM8_CH2`) and supports one-press emergency stop via `PC13`.
