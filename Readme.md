<div align="center">

# Industrial Motor Health Monitoring, Fault Detection & Closed-Loop Control System

**Edge-Intelligent Motion Control & Predictive Maintenance Node powered by STM32G491RE (ARM Cortex-M4 @ 170 MHz)**

[![MCU](https://img.shields.io/badge/MCU-STM32G491RE%20(Cortex--M4)-03234B?style=for-the-badge&logo=stmicroelectronics)](https://www.st.com/en/microcontrollers-microprocessors/stm32g491re.html)
[![Language](https://img.shields.io/badge/Language-Embedded%20C%20(C99)-A8B9CC?style=for-the-badge&logo=c)](https://en.cppreference.com/w/c)
[![DSP](https://img.shields.io/badge/Edge%20DSP-ARM%20CMSIS--DSP-0091BD?style=for-the-badge&logo=arm)](https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html)
[![License](https://img.shields.io/badge/Status-Phase%201--7%20Verified-2EA44F?style=for-the-badge)](#)

[System Architecture](#-system-architecture) • [Hardware & Pinout](#-hardware-bill-of-materials--pinout) • [Measured Benchmarks](#-experimental-validation--benchmarks) • [Gallery & Demos](#-hardware-gallery--live-demonstrations) • [Industrial Roadmap](#-industrial-extension-roadmap-jan--apr-2027)

</div>

---

## 🎯 Executive Summary & Problem Statement

Conventional low-voltage industrial motor controllers typically operate as open-loop binary actuators (`ON/OFF`), offering zero diagnostic visibility into the physical health of the drive train. Incipient mechanical and electrical anomalies—such as shaft unbalance, progressive bearing wear, supply voltage droop, or sudden load binding—often escalate into catastrophic winding burnout and unplanned factory downtime.

Rather than relying on high-latency cloud pipelines that consume prohibitive bandwidth streaming raw sensor data, this project implements a **deterministic, edge-intelligent embedded node** on an **STM32G491RE**. It unifies:
1. **Closed-Loop Speed Regulation:** A $100\text{ Hz}$ discrete PI controller with anti-windup saturation clamping that locks shaft speed under variable mechanical torque.
2. **Synchronous Multi-Parametric Sensing:** Real-time acquisition of rotational speed, armature current, DC bus voltage, and 3-axis casing vibration.
3. **Silicon-Level Edge Analytics:** On-chip vector signal processing using **ARM CMSIS-DSP** (Real FFT & windowed RMS) to compute an empirical **0–100 Composite Health Index** and enforce autonomous safety cutoffs ($<50\text{ ms}$ stall/overcurrent trip).

---

## 🏗️ System Architecture

```text
+---------------------------------------------------------------------------------------------------+
|                                   PHYSICAL PLANT & POWER STAGE                                    |
|                                                                                                   |
|  [19.5V Supply] ---> [Buck Converter (12.09V)] ---> [5A Fuse & Switch] ---> [BTS7960 43A Bridge]  |
|                                                                                    |              |
|                                                                                    v              |
|  [MPU6050 6-DOF IMU] <--- (Mounted on Casing) <--- [N25 12V Geared DC Motor + 334 PPR Encoder]    |
|           |                                                |                       |              |
+-----------|------------------------------------------------|-----------------------|--------------+
            | I2C (400 kHz)                                  | In-Line Current       | Quadrature
            |                                                v                       | A/B Pulses
            |                                       [ACS712-05B Sensor]              |
            v                                                |                       v
+---------------------------------------------------------------------------------------------------+
|                           STM32G491RE EDGE CONTROLLER (ARM Cortex-M4F @ 170 MHz)                  |
|                                                                                                   |
|  +-----------------------+    +-------------------------+    +---------------------------------+  |
|  |   Sensing Front-End   |    |    Control & Safety     |    |     On-Chip Edge Analytics      |  |
|  |                       |    |                         |    |                                 |  |
|  | • TIM2: 4x Encoder    |--->| • 100 Hz Discrete PI    |--->| • CMSIS-DSP Real FFT (FPU)      |  |
|  | • ADC2: Current + EMA |--->| • Anti-Windup Clamping  |--->| • Windowed RMS & Peak Metrics   |  |
|  | • ADC1: 4:1 Bus Sense |--->| • <50 ms Stall/OC Trip  |--->| • 0–100 Health Score Engine     |  |
|  +-----------------------+    +-------------------------+    +---------------------------------+  |
|                                            |                                   |                  |
+--------------------------------------------|-----------------------------------|------------------+
                                             |                                   |
                                             v                                   v
                               [TIM3/TIM4 PWM & Enable Lines]      [16x2 LCD & UART Telemetry]
```

---

## 🔧 Hardware Bill of Materials & Pinout

| Subsystem | Component / Module | Technical Specification | STM32G491RE Interface |
| :--- | :--- | :--- | :--- |
| **Microcontroller** | STMicroelectronics NUCLEO-G491RE | ARM Cortex-M4 @ 170 MHz, Hardware FPU, 512 KB Flash, 96 KB SRAM | Core MCU |
| **Actuator** | TT Motor N25 12V 370 RPM | Metal-gear DC motor with integrated 334 PPR Hall quadrature encoder | `TIM2_CH1` (PA0), `TIM2_CH2` (PA1) |
| **Motor Driver** | BTS7960B Half-Bridge | 43A high-current dual H-Bridge with optical/logic isolation | `TIM3_CH2` (PC7), `TIM4_CH1` (PB6), `PC8`/`PC9` |
| **Current Sensor** | Allegro ACS712-05B | $\pm 5\text{ A}$ Hall-effect current sensor ($185\text{ mV/A}$ sensitivity) | `ADC2_IN17` (64-sample moving average) |
| **Voltage Sense** | Precision Resistor Divider | $30\text{ k}\Omega / 10\text{ k}\Omega$ (4:1 ratio) + $100\text{ nF}$ ceramic low-pass filter | `ADC1` Analog Input |
| **Vibration IMU** | InvenSense MPU6050 | 3-axis MEMS accelerometer ($\pm 2g$ range, gravity decoupled) | Fast-Mode I²C Bus (`0x68`) |
| **Power & Safety** | Dell 19.5V Brick + XL4015 Buck | Calibrated $12.09\text{ V}$ DC bus, inline $5\text{ A}$ fast-blow fuse, master rocker switch | Common Star-Ground Reference |
| **Local UI** | HD44780 16×2 LCD & ST-LINK UART | Multi-page live health dashboard & $115200\text{ bps}$ serial telemetry | GPIO Parallel & `USART2` |

---

## 📊 Experimental Validation & Benchmarks

All control and sensing subsystems have been physically verified on the hardware test rig:

| Test Scenario | Target Setpoint | Measured Speed | PWM Duty Cycle | Armature Current | System Response |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **No-Load Free Run** | `150.0 RPM` | `149.8 RPM` | `29.5%` (295/999) | `+0.16 A` | Steady-state lock within $\pm 1.5\text{ RPM}$ |
| **Dynamic Mechanical Drag** | `150.0 RPM` | `148.5 RPM` | `47.4% – 53.3%` | `+0.38 A` | PI integral action compensates torque & restores RPM |
| **Static IMU Calibration** | Stationary | `0.0 RPM` | `0.0%` | `0.00 A` | `X: +0.01g`, `Y: -0.04g`, `Z: +1.02g` ($1g$ gravity verified) |
| **Locked-Rotor / Stall** | `150.0 RPM` | `0.0 RPM` | `0.0%` (Tripped) | Spike $> \text{Limit}$ | Autonomous cutoff triggered in $< 50\text{ ms}$ |

---

## 📸 Hardware Gallery & Live Demonstrations

> **Note for Evaluators:** Add your captured bench photos to `media/images/` and test recordings to `media/videos/` using the exact filenames below so they render automatically on this page.

### Physical Hardware & Telemetry Captures

| Complete Test Rig & Power Stage | Breadboard Signal Conditioning & Dividers | Live Serial Telemetry & LCD Dashboard |
| :---: | :---: | :---: |
| <img src="media/images/hardware_rig_topdown.png" alt="Hardware Rig" width="280"/> | <img src="media/images/breadboard_circuit.png" alt="Signal Conditioning" width="280"/> | <img src="media/images/telemetry_dashboard.png" alt="Live Telemetry" width="280"/> |
| *NUCLEO-G491RE, BTS7960, N25 Motor & Buck Stage* | *30k/10k Bus Divider & ACS712 Filtering* | *115200 baud UART & 16×2 Health Display* |

### Verification Video Logs
* 🎬 **[Phase 1: Open-Loop H-Bridge & PWM Actuation](media/videos/phase1_motor_spin_test.mp4)** — Demonstrates bidirectional PWM control, coasting states, and hardware fuse/switch isolation.
* 🎬 **[Phase 4: Closed-Loop PI Load Rejection Test](media/videos/phase4_load_regulation_drag_test.mp4)** — Physical friction applied to the rotating shaft; demonstrates real-time current surge tracking ($+0.16\text{ A} \rightarrow +0.38\text{ A}$) and speed recovery to $150\text{ RPM}$.
* 🎬 **[Phase 6–7: Multi-Sensor Fusion & Vibration Analytics](media/videos/phase6_closedloop_vibration_demo.mp4)** — Simultaneous speed lock, current sensing, and live 3-axis MPU6050 vibration monitoring.

---

## 📂 Repository Structure

```text
Industrial-Motor-Controller-and-Health-Monitoring-System/
├── Motor_Health_Phase1/          # STM32CubeIDE Firmware Project
│   ├── Core/
│   │   ├── Inc/                  # Peripheral & DSP header definitions
│   │   └── Src/
│   │       └── main.c            # 100 Hz PI loop, ADC/I2C pipelines, LCD & Fault logic
│   ├── Drivers/                  # STM32G4xx HAL/LL & ARM CMSIS-DSP libraries
│   └── Motor_Health_Phase1.ioc   # STM32CubeMX hardware pinout & clock configuration
├── media/
│   ├── images/                   # Bench setup photos, wiring & oscilloscope captures
│   └── videos/                   # Physical load testing & fault trip demonstrations
└── README.md                     # Master engineering documentation
```

---

## 🚀 Industrial Extension Roadmap (Jan – Apr 2027)

To transition this validated bench prototype into a production-grade industrial/automotive ECU without discarding existing hardware, the following bare-metal and hardware engineering upgrades are scheduled:

* **Custom 2-Layer Stacking Shield PCB (KiCad):** Consolidate all jumper wiring onto a custom Nucleo-64 Morpho/Arduino carrier shield featuring star-ground partitioning (separating high-current motor ground from analog sensor ground), BAT54S Schottky clamping, and SMAJ15CA TVS transient suppression.
* **Sub-Microsecond Hardware Protection (`TIM1` + `COMP1/2`):** Migrate PWM actuation to Advanced Timer `TIM1` (`PA8`/`PA9`) and link internal dual analog comparators in window mode directly to `TIM1_BDTR` break logic for $<1\ \mu\text{s}$ autonomous hardware gate shutdown—bypassing CPU latency entirely.
* **Zero-CPU DMA Pipeline:** Synchronize `ADC1`/`ADC2` conversions to the midpoint trigger (`TIM1_TRGO`) using circular ping-pong DMA buffers to eliminate MOSFET switching noise and free CPU cycles.
* **Industrial Fieldbuses (Modbus-RTU & Classic CAN 2.0B):** Implement a deterministic Modbus-RTU slave engine ($t_{3.5}$ idle framing, CRC-16) over RS-485 (`SP3485`) and $500\text{ kbps}$ cyclic CAN telemetry via on-chip `FDCAN1` (`SN65HVD230`).
* **Fail-Safe A/B Bootloader & IEC 60730 Class-B Self-Tests:** Deploy a dual-slot In-Application Programming (IAP) bootloader (`VTOR` relocation, 64-bit append-only flash metadata, 3-strike crash rollback), $200\text{ ms}$ pre-fault freeze-frame logging, and startup CPU/SRAM/Flash integrity checks.

---

<div align="center">

**Developed by Shashwat** | B.Tech Electronics & Communication Engineering  
*Built for Embedded Firmware, Motor Control, and Hardware-Software Co-Design Applications*

</div>