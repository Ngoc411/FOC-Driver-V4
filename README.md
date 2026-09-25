# FOC Driver V4

Firmware for a custom **BLDC motor-control board** based on the **STM32G474RET6 MCU**. The project implements **Field-Oriented Control (FOC)** with **Space Vector PWM (SVPWM)** and provides three independent control modes: **current, speed, and position control**.

The controller is designed as a compact motor-control platform for robotic actuators and other motion-control applications. Motor position is measured using an **MT6816 magnetic encoder**, while an external controller can communicate with the board through **Classical CAN**.

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

The firmware provides three **independent control modes**, all based on **PID control**. The user selects the required mode depending on the application.

In **current control**, a PID controller regulates the commanded current, providing direct control of the motor's torque-producing current.

In **speed control**, the commanded speed is compared with the measured motor speed from the encoder. The **speed PID controller** generates a current command, which is then passed directly to the current-control loop.

In **position control**, the commanded position is compared with the encoder position. The **position PID controller** generates a current command, which is also passed directly to the current-control loop.

The three modes share the same current-control and FOC/SVPWM stages, while **speed and position control independently generate the current command** without an intermediate control layer.

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

| Component       | Description                 |
| --------------- | --------------------------- |
| MCU             | STM32G474RET6               |
| Motor           | 3-phase BLDC                |
| Encoder         | MT6816 magnetic encoder     |
| Gate Driver     | DRV8323S                    |
| CAN Transceiver | SN65HVD230                  |
| Communication   | Classical CAN, up to 1 Mbps |

---

## Firmware Structure

The firmware is organized into dedicated libraries for motor control, encoder feedback, CAN communication, driver configuration, and other supporting functions. These modules are integrated by the main application in `Core/Src/main.c`, which handles initialization and coordinates the overall control flow.

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

## Results

The following videos demonstrate the motor controller operating in the different control modes.

### Current Control

Demonstration of direct current control and the resulting motor response.

https://github.com/Ngoc411/FOC-Driver-V4/blob/main/current_ctrl.mov

### Speed Control

Demonstration of closed-loop speed control using feedback from the MT6816 encoder.

https://github.com/Ngoc411/FOC-Driver-V4/blob/main/speed_ctrl.mov
### Position Control

Demonstration of closed-loop position control using the MT6816 encoder.

https://github.com/Ngoc411/FOC-Driver-V4/blob/main/position_ctrl.mov
## Author

**Ngoc**

Embedded Systems · Motor Control · Robotics
