# FOC Driver V4 Firmware for a custom **BLDC motor-control board** based on the **STM32G474RET6 MCU**.

The project implements **Field-Oriented Control (FOC)** with **Space Vector PWM (SVPWM)** and provides three independent control modes: **current, speed, and position control**. The controller is designed as a compact motor-control platform for robotic actuators and other motion-control applications. Motor position is measured using an **MT6816 magnetic encoder**, while an external controller can communicate with the board through **Classical CAN**.

---

## Features

* **BLDC motor control** using FOC + SVPWM
* Three independent control modes:
  * **Current control** — A
  * **Speed control** — rad/s
  * **Position control** — rad
* **MT6816 magnetic encoder** for rotor position feedback
* **20 kHz motor-control loop**
* **Classical CAN** communication up to **1 Mbps**
* **DRV8323S** three-phase gate driver
* Hardware PWM generation using STM32 timers
* Current sensing and closed-loop current control
* Designed for integration with external controllers and robotic systems

<h3 align="center"> Control Mode Demonstrations</h3>

<p align="center">
  <em>Field-Oriented Control — three independent control modes on the STM32G474RET6.</em>
</p>

---

#### Current Control
<p align="center">
  <img src="https://github.com/Ngoc411/FOC-Driver-V4/blob/main/current_ctrl.gif" width="640"><br>
  <sub>Direct torque-current regulation</sub>
</p>

---

#### Speed Control
<p align="center">
  <img src="https://github.com/Ngoc411/FOC-Driver-V4/blob/main/speed_ctrl.gif" width="640"><br>
  <sub>Closed-loop speed regulation using MT6816 feedback</sub>
</p>

---

#### Position Control
<p align="center">
  <img src="https://github.com/Ngoc411/FOC-Driver-V4/blob/main/pos_ctrl.gif" width="640"><br>
  <sub>Closed-loop position regulation using MT6816 feedback</sub>
</p>

---

## System Architecture

```text
                External Controller
                 MCU / Jetson / PC
                        │
                   CAN Bus
                        │
                        ▼
              ┌───────────────────┐
              │   STM32G474RET6   │
              │                   │
              │ Position Control  │
              │ Speed Control     │
              │ Current Control   │
              │        ↓          │
              │       FOC         │
              │        ↓          │
              │      SVPWM        │
              └─────────┬─────────┘
                        │
                   3-Phase PWM
                        │
                        ▼
                    DRV8323S
                        │
                        ▼
                 3-Phase Inverter
                        │
                        ▼
                   BLDC Motor
                        ▲
                        │
                   MT6816 Encoder
```

---

## Control Architecture

The firmware provides three **independent control modes**, with PID-based regulation used throughout the control system. The required mode is selected according to the application. In **current control**, the commanded current is regulated directly to control the motor's torque-producing current. In **speed control**, the commanded speed is compared with the encoder feedback, and the resulting control output is converted into a current command for the current-control stage. In **position control**, the commanded position is compared with the encoder feedback, and the resulting control output is directly converted into a current command. All three modes share the same current-control, FOC, and SVPWM stages. Speed and position control operate independently and both produce the current command directly, with no intermediate speed-control layer between position and current control.

```text
┌──────────────────┐   ┌──────────────────┐   ┌──────────────────┐
│ Current Control  │   │  Speed Control   │   │ Position Control │
│                  │   │                  │   │                  │
│ Current Command  │   │  Speed Command   │   │ Position Command │
│       │          │   │       │          │   │       │          │
└────────┬─────────┘   └───────┬──────────┘   └───────┬──────────┘
         └─────────────────────┼──────────────────────┘
                               ▼
                        Current Command
                               │
                               ▼
                         FOC → SVPWM
                               │
                               ▼
                             Motor
```

The common FOC stage converts the desired current into the required motor voltage vector and generates the corresponding three-phase PWM signals through SVPWM.

---

## Hardware

| Component | Description |
| --------------- | --------------------------- |
| MCU | STM32G474RET6 |
| Motor | 3-phase BLDC |
| Encoder | MT6816 magnetic encoder |
| Gate Driver | DRV8323S |
| CAN Transceiver | SN65HVD230 |
| Communication | Classical CAN, up to 1 Mbps |

---

## Firmware Structure

The firmware is organized into dedicated libraries for motor control, encoder feedback, CAN communication, driver configuration, and other supporting functions. These modules are integrated by the main application in Core/Src/main.c, which handles initialization and coordinates the overall control flow.

```text
FOC-Driver-V4/
│
├── Core/
│   └── Src/
│       └── main.c
│
├── USB_Device/
│
├── lib/
│   ├── CAN/
│   ├── DRV8323/
│   ├── FLASH/
│   ├── FOC/
│   ├── MT6816/
│   ├── Constants/
│   └── ...
│
├── Drivers/
├── Middlewares/
└── ...
```

The modular structure allows the motor-control algorithms and hardware interfaces to be developed and maintained independently.

---

## Author

**Ngoc**

Embedded Systems · Motor Control · Robotics
