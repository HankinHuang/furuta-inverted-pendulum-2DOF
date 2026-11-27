# Mode Switching Logic & Custom PCB Integration

Summary of the embedded mode-switching state machine and custom Nucleo-F446RE shield PCB documented in Sections 4.6 and 5.3 of [`docs/Final_Report_G2025.pdf`](../../docs/Final_Report_G2025.pdf).

## 1. Swing-Up to Balance Switching Logic

The swing-up and upright-balancing controllers run as mutually exclusive operating modes:

1. **Idle / Stopped**: Motor disabled (`PWM = 0`). Pressing the blue user button (`PC13`) zeroes the encoder reference and enters **Swing-Up Mode**.
2. **Swing-Up -> Balance Transition**: Control transfers to the balancing controller (PID or LQR/Soft-LQR) only when all three conditions hold for **10 consecutive `1 ms` samples** (`10 ms` debounce):
   - $|\theta_p| < \theta_{\text{th}}$ (selected as **`0.25 rad`** after achieving 100% switching success over 20 trials without post-switch PWM saturation)
   - $|\dot{\theta}_p| < 5.0\text{ rad/s}$
   - $|\dot{\theta}_a| < 5.0\text{ rad/s}$
3. **Fault / Stop Latch**: If balance is lost or `PC13` is pressed during active control, motor output is disabled and latched until manual restart.

## 2. Custom Shield PCB Power & Interface Design

To eliminate breadboard jumper-wire faults during dynamic testing, a two-layer shield PCB for the **STM32 Nucleo-F446RE** integrates:

- **12 V -> 5 V Rail**: `XL1509-5.0` fixed 5 V buck regulator (`47 uH` inductor, `220 uF/35 V` bulk caps, `SS34` Schottky diode) powering the pendulum encoder.
- **12 V -> 3.3 V Rail**: `MP4462` adjustable buck regulator ($R_{11} = 124\text{ k}\Omega$, $R_{12} = 40.2\text{ k}\Omega \implies V_{\text{out}} \approx 3.27\text{ V}$) powering the 3.3 V logic rail and `4.7 kOhm` pull-ups for the NPN open-collector pendulum encoder (`A/B/Z`).
- **Onboard Motor Driver & Inputs**: Socketed `TB6612FNG` H-bridge footprint and four debounced push-button inputs.
