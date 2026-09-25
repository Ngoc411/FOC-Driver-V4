# FOC Driver V4

Firmware for a custom **BLDC motor-control board** based on the **STM32G474RET6 MCU**. The project implements **Field-Oriented Control (FOC)** with **Space Vector PWM (SVPWM)** and supports current, speed, and position control.

The board is designed as a compact motor-control platform for robotic actuators and other motion-control applications.

---

## Features

* **BLDC motor control** using FOC + SVPWM
* Three control modes:

  * **Current control** — A
  * **Speed control** — rad/s
  * **Position control** — rad
* **MT6816 magnetic encoder** for rotor position feedback
* **10 kHz motor-control loop**
* **Classical CAN** communication up to **1 Mbps**
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
              │ Position Control/ │
              │  Speed Control/   │
              │ Current Control/  │
              │         ↓         │
              │       FOC         │
              │         ↓         │
              │     SVPWM         │
              └─────────┬─────────┘
                        │
                   3-Phase PWM
                        │
                        ▼
                 Gate Driver /
                 Power Stage
                        │
                        ▼
                   BLDC Motor
                        ▲
                        │
                   MT6816 Encoder
```

---

## Control Architecture

## Control Architecture

The firmware provides three independent control modes. Each mode generates a current command that is passed directly to the current controller.

```text
Current Command ───────────────┐
                               │
Speed Command → Speed Control ─┤
                               ├──→ Current Control → FOC → SVPWM → Motor
Position Command → Position ───┤
                    Control    │
                               │
```

This allows the same controller to operate either as a direct torque/current controller or as a higher-level position-controlled actuator.

---

## Hardware

| Component         | Description             |
| ----------------- | ----------------------- |
| MCU               | STM32G474RET6           |
| Motor             | 3-phase BLDC            |
| Encoder           | MT6816 magnetic encoder |
| Driver            | DRV8323S                |
| Communication     | SN65HVD230              |

---

## Firmware Structure

The firmware is organized into separate modules for motor control, feedback, communication, and hardware drivers.

```text
FOC-Driver-V4/
│
├── Core/
├── USB_Device/
├── lib/
│   ├── CAN/
│   ├── MT6816/
│   └── Coefficients/
│   └── ...
│
└── ...
```

The modular structure makes it easier to maintain the motor-control algorithm and integrate additional peripherals or control features.

---

## Applications

The controller is intended for applications such as:

* Robotic actuators
* Humanoid robots
* Multi-axis motion systems
* BLDC servo systems
* General embedded motor-control applications

---

## Technologies

`C` · `STM32G4` · `FOC` · `SVPWM` · `BLDC` · `CAN` · `SPI` · `ADC` · `PWM` · `MT6816`

---

## Author

**Ngoc**

Embedded Systems · Motor Control · Robotics
