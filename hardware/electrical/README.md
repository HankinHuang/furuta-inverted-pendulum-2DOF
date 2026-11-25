# Electrical Hardware & Bill of Materials (BOM)

This directory documents the electrical subsystem, Bill of Materials (BOM), and microcontroller wiring for the 2-DOF Furuta Inverted Pendulum platform.

- English wiring & architecture notes: [hardware_notes.md](hardware_notes.md)
- Chinese companion documents: [README_CN.md](README_CN.md), [hardware_notes_CN.md](hardware_notes_CN.md)

---

## 1. Deployed STM32F446RE Pinout & Peripheral Mapping

Verified against `firmware/swingup_lqr/Core/Src/main.c` and `firmware/swingup_pid/Core/Src/main.c`:

| Function | Peripheral / Pin | Notes |
| :--- | :--- | :--- |
| **Motor PWM Output** | `TIM8_CH2` | Period `4199`, software duty safety cap `0.80` (`12.0 V` bus) |
| **Motor Direction Control** | `PA10`, `PB3` | Complementary digital direction outputs (`GPIO_OUTPUT_PP`) |
| **Motor Shaft Encoder** | `TIM1` (`TI12` 4x quadrature) | `44` counts/rev at motor shaft (`9.267:1` gear ratio) |
| **Pendulum Pivot Encoder** | `TIM3` (`TI12` 4x quadrature) | `4096` counts/rev routed through conductive slip ring |
| **Serial Telemetry** | `USART2` (`115200` baud, 8N1) | Diagnostics & state telemetry every `50 ms` |
| **User Start / E-Stop Button** | `PC13` (`B1_Pin`, `EXTI15_10`) | Falling-edge interrupt toggles swing-up start and emergency stop |

---

## 2. Bill of Materials (Converted from `electrical_components.doc`)

### 2.1 Core Platform & PCB Integration Components

| Component | Model / Part No. | Specification / Role | Status |
| :--- | :--- | :--- | :--- |
| **Microcontroller Board** | STM32 Nucleo-F446RE | Primary 1 kHz real-time controller | Confirmed |
| **DC Motor Driver** | TB6612FNG | Dual H-bridge PWM + direction motor driver | Confirmed |
| **DC-DC Buck Regulator** | MP1584EN | Steps down supply rail to 5 V / 3.3 V logic power | Confirmed |
| **Power Adapter** | 12 V / 3 A DC supply | Main motor & logic bus power | Confirmed |
| **Conductive Slip Ring** | 6-circuit hollow shaft (ID 5 mm) | Continuous 360° arm rotation without cable twist | Confirmed |
| **Logic Level Converter** | Bidirectional level shifter | Interfaces 5 V encoder signals to 3.3 V STM32 GPIO | Confirmed |
| **SWD Debugger** | ST-LINK | Firmware flashing & live debug (onboard / external) | Confirmed |
| **USB-to-UART Converter** | CH340 | Auxiliary serial debugging | Confirmed |
| **Rotary Potentiometer** | RV0931-B103 | Panel-mount tuning/reference input | Confirmed |
| **OLED Display (Optional)** | 0.96" I2C OLED (4-pin) | `GND / VCC / SCL / SDA` status display | Confirmed |
| **PCB Power & Debug Headers** | DC barrel jack, SS-12D00G3 switch, screw terminal, SWD & UART headers, Dupont wires | Custom PCB integration and harness wiring | Confirmed |

### 2.2 Earlier / Alternative Evaluated Components

The following items were also recorded in the team's component evaluation log prior to finalizing the 12 V DC geared motor + TB6612FNG configuration:

| Component | Model / Part No. | Notes |
| :--- | :--- | :--- |
| **Alternative MCU** | STM32F103C8T6 (Blue Pill) | Replaced by Nucleo-F446RE for FPU performance and hardware timers |
| **Alternative Brushless Drive** | DJI RoboMaster M2006 P36 + C610 ESC + SN65HVD230 CAN transceiver | Evaluated 24 V CAN brushless option |
| **Alternative Optical/Capacitive Encoder** | AMT103-V | Evaluated rotary encoder option |
