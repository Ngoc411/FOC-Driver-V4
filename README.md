# FOC Driver V4 — BLDC Motor Control Firmware

Firmware for a custom BLDC motor-control board based on the **STM32G474RET6**, implementing **Field-Oriented Control (FOC)** with **Space Vector Pulse Width Modulation (SVPWM)**.

The system is designed as a compact motor-control platform capable of operating a BLDC motor in **current, speed, and position control modes**, while providing communication with an external controller through **Classical CAN**.

The project was developed with a focus on real-time motor control, modular firmware architecture, encoder feedback, and reliable communication between the motor controller and higher-level systems.

---

## Features

* **BLDC motor control using FOC**
* **SVPWM-based three-phase inverter control**
* Three closed-loop control modes:

  * Current control
  * Speed control
  * Position control
* Magnetic encoder feedback using **MT6816**
* **10 kHz motor-control loop**
* **Classical CAN communication**
* CAN communication speed up to **1 Mbps**
* STM32G474RET6 MCU
* Hardware-based PWM generation using STM32 timers
* Current sensing for closed-loop torque/current control
* Modular firmware structure for motor control and peripheral drivers
* Designed for integration with an external robot or motion-control system

---

# 1. System Overview

The firmware runs on a custom motor-control board built around the **STM32G474RET6**, which is an STM32G4-series MCU designed for real-time digital motor control.

The overall control architecture can be summarized as:

```text
                 ┌───────────────────────┐
                 │   External Controller  │
                 │  MCU / Jetson / PC    │
                 └───────────┬───────────┘
                             │
                       Classical CAN
                             │
                             ▼
                 ┌───────────────────────┐
                 │    STM32G474RET6      │
                 │                       │
                 │  CAN Communication    │
                 │          │            │
                 │          ▼            │
                 │   Control Algorithm   │
                 │   ┌───────────────┐   │
                 │   │ Current       │   │
                 │   │ Speed         │   │
                 │   │ Position      │   │
                 │   └───────┬───────┘   │
                 │           │           │
                 │           ▼           │
                 │         FOC           │
                 │           │           │
                 │           ▼           │
                 │        SVPWM          │
                 └───────────┬───────────┘
                             │
                       3-Phase PWM
                             │
                             ▼
                    ┌────────────────┐
                    │ Gate Driver /  │
                    │ 3-Phase Bridge │
                    └───────┬────────┘
                            │
                            ▼
                      BLDC Motor
                            ▲
                            │
                      MT6816 Encoder
                            │
                            └──────────────
```

The STM32 receives commands from an external controller, processes the selected control mode, reads motor feedback, calculates the required voltage vector, and generates the corresponding PWM signals for the three-phase inverter.

---

# 2. Hardware

## Main MCU

**STM32G474RET6**

The STM32G474RET6 is responsible for:

* Real-time motor-control calculations
* PWM generation
* ADC-based current measurement
* Encoder communication
* CAN communication
* Current control
* Speed control
* Position control
* Protection and fault handling

The STM32G4 family is particularly suitable for this application because it provides peripherals and computational resources intended for digital power conversion and motor-control applications.

---

## Motor

The firmware is designed for controlling a three-phase BLDC/PMSM-type motor using sinusoidal commutation through FOC.

The controller operates the motor through the three electrical phases:

```text
             Phase A
                │
                │
           ┌────┴────┐
           │         │
           │  Motor  │
           │         │
           └────┬────┘
              /   \
             /     \
        Phase B   Phase C
```

---

## Encoder

Motor position feedback is provided by the **MT6816 magnetic encoder**.

The encoder provides rotor position information required by the FOC algorithm.

The measured mechanical position is used to determine the electrical rotor angle:

```text
Mechanical Position
        │
        ▼
MT6816 Encoder
        │
        ▼
Rotor Angle
        │
        ▼
Electrical Angle
        │
        ▼
Clarke / Park Transform
        │
        ▼
FOC Controller
```

Accurate rotor-angle information is critical for FOC because the controller must align the current vector with the rotor magnetic field.

---

## Gate Driver and Power Stage

The motor-control board uses a three-phase MOSFET inverter driven by a dedicated gate-driver stage.

The STM32 generates three-phase PWM signals, which are provided to the gate-driver circuit.

The basic power architecture is:

```text
DC Bus
  │
  ▼
3-Phase MOSFET Bridge
  │
  ├── Phase A ──┐
  ├── Phase B ──┼──> BLDC Motor
  └── Phase C ──┘
```

The firmware controls the inverter through SVPWM.

---

# 3. Field-Oriented Control

The core of the project is **Field-Oriented Control (FOC)**.

Unlike simple six-step commutation, FOC treats the motor currents as a rotating vector and controls the torque-producing and flux-producing components independently.

The main processing chain is:

```text
Phase Currents
      │
      ▼
Clarke Transform
      │
      ▼
   Iα / Iβ
      │
      ▼
Park Transform
      │
      ▼
   Id / Iq
      │
      ▼
Current Controllers
      │
      ▼
   Vd / Vq
      │
      ▼
Inverse Park Transform
      │
      ▼
 Vα / Vβ
      │
      ▼
     SVPWM
      │
      ▼
Three-Phase PWM
```

---

# 4. Clarke Transform

The measured three-phase currents are transformed from the stationary three-phase coordinate system into the two-axis stationary reference frame:

```text
Ia, Ib, Ic
    │
    ▼
Clarke Transform
    │
    ▼
Iα, Iβ
```

This reduces the three-phase current system to two orthogonal components while preserving the information required for motor control.

---

# 5. Park Transform

The stationary-frame current vector is then transformed into a rotating reference frame aligned with the rotor:

```text
Iα / Iβ
   │
   ▼
Park Transform
   │
   ├── Id
   │
   └── Iq
```

Where:

* **Id** represents the flux-producing component.
* **Iq** represents the torque-producing component.

For normal operation, the controller primarily regulates **Iq** to control motor torque while maintaining the desired flux condition through **Id**.

---

# 6. Current Control

The lowest-level closed-loop controller regulates the motor current.

The controller compares the measured current with the target current:

```text
Target Iq
   │
   ▼
 ┌───────┐
 │ Error │◄──────── Measured Iq
 └───┬───┘
     │
     ▼
 PI Controller
     │
     ▼
   Vq
```

The same principle can be applied to the d-axis current.

The current loop operates at a high update rate to provide fast torque response.

### Current Control Unit

```text
A (Ampere)
```

The current command represents the desired motor torque-producing current.

---

# 7. Speed Control

The firmware also supports closed-loop speed control.

The speed controller compares the target motor speed with the measured speed obtained from the encoder:

```text
Target Speed
     │
     ▼
 ┌──────────┐
 │   Error  │◄──── Measured Speed
 └────┬─────┘
      │
      ▼
 Speed Controller
      │
      ▼
 Current / Torque Command
      │
      ▼
 Current Control
      │
      ▼
     FOC
```

This creates a cascaded control structure where the speed controller generates a current/torque command for the inner current loop.

### Speed Control Unit

```text
rad/s
```

---

# 8. Position Control

The highest-level control mode is position control.

The controller compares the desired motor position with the measured encoder position:

```text
Target Position
      │
      ▼
 ┌──────────┐
 │ Position │
 │  Error   │◄──── Encoder Position
 └────┬─────┘
      │
      ▼
Position Controller
      │
      ▼
 Speed Command
      │
      ▼
Speed Controller
      │
      ▼
Current Controller
      │
      ▼
     FOC
```

This produces a cascaded position → speed → current control architecture.

### Position Control Unit

```text
rad
```

---

# 9. Control Modes

The firmware therefore provides three different levels of motor control:

| Control Mode     | Command Unit | Main Purpose                    |
| ---------------- | -----------: | ------------------------------- |
| Current Control  |            A | Direct torque/current control   |
| Speed Control    |        rad/s | Regulate motor rotational speed |
| Position Control |          rad | Regulate motor position         |

The control modes allow the same motor-control hardware to be used for different applications, from direct torque control to robotic joint control.

---

# 10. SVPWM

The controller uses **Space Vector Pulse Width Modulation (SVPWM)** to convert the desired stator voltage vector into three-phase PWM duty cycles.

The FOC controller produces the desired voltage vector:

```text
Vd / Vq
   │
   ▼
Inverse Park
   │
   ▼
Vα / Vβ
   │
   ▼
 SVPWM
   │
   ├── Duty A
   ├── Duty B
   └── Duty C
```

These duty cycles are then generated using the STM32 hardware timers.

SVPWM provides efficient utilization of the available DC-bus voltage and is well suited for three-phase motor control.

---

# 11. Control Loop

The motor-control firmware is designed around a high-frequency real-time control loop.

The main sequence is conceptually:

```text
┌─────────────────────────────┐
│      Control Interrupt      │
└──────────────┬──────────────┘
               │
               ▼
        Read ADC Currents
               │
               ▼
       Read Encoder Angle
               │
               ▼
        Calculate Speed
               │
               ▼
      Clarke Transformation
               │
               ▼
       Park Transformation
               │
               ▼
       Current Controller
               │
               ▼
    Inverse Park Transformation
               │
               ▼
             SVPWM
               │
               ▼
        Update PWM Duty
               │
               ▼
       Next Control Cycle
```

The time-critical motor-control calculations are executed independently from lower-priority communication and background tasks.

---

# 12. CAN Communication

The motor controller communicates with external devices using **Classical CAN**.

### Communication specification

| Parameter          | Specification       |
| ------------------ | ------------------- |
| Protocol           | Classical CAN       |
| Maximum Bit Rate   | 1 Mbps              |
| Physical Interface | CAN                 |
| Data Payload       | Up to 8 bytes/frame |

CAN allows the motor controller to operate as a distributed actuator node in a larger robotic or motion-control system.

For example:

```text
             ┌──────────────────┐
             │ Main Controller  │
             │ MCU / Computer   │
             └────────┬─────────┘
                      │
                   CAN Bus
                      │
          ┌───────────┼───────────┐
          │           │           │
          ▼           ▼           ▼
      Motor 1     Motor 2     Motor 3
      Driver      Driver      Driver
```

This architecture allows multiple motor-control boards to share a common communication bus.

---

# 13. Firmware Architecture

The firmware is organized into separate modules to keep hardware drivers, communication, and motor-control algorithms independent.

A simplified architecture is:

```text
Application
    │
    ├── Motor Control
    │     ├── Position Control
    │     ├── Speed Control
    │     └── Current Control
    │
    ├── FOC
    │     ├── Clarke Transform
    │     ├── Park Transform
    │     ├── Inverse Park
    │     └── SVPWM
    │
    ├── Feedback
    │     ├── Current Measurement
    │     └── MT6816 Encoder
    │
    ├── Communication
    │     └── CAN
    │
    └── Hardware Drivers
          ├── PWM / Timer
          ├── ADC
          ├── SPI
          └── GPIO
```

This separation makes the firmware easier to maintain and allows individual components to be tested independently.

---

# 14. Real-Time Design

Motor control is a real-time application where deterministic timing is important.

The firmware therefore separates:

### High-priority tasks

* Current measurement
* Encoder acquisition
* FOC calculations
* Current control
* SVPWM update
* PWM generation

### Lower-priority tasks

* CAN communication
* Command processing
* Status reporting
* Diagnostics
* Non-time-critical background processing

This architecture prevents communication or diagnostic operations from unnecessarily delaying the time-critical motor-control loop.

---

# 15. Hardware Protection

The motor controller is intended to operate a relatively high-current three-phase inverter, so protection mechanisms are an important part of the system.

The firmware can be integrated with hardware fault signals from the gate-driver and power stage.

Typical fault conditions that need to be handled include:

* Over-current
* Gate-driver fault
* Invalid encoder feedback
* Loss of communication
* Motor-control fault
* Emergency stop conditions

Hardware-level protection is preferred for critical power-stage faults because it can disable the inverter independently of firmware execution.

---

# 16. Development Environment

The project is developed for the STM32G4 platform using the STM32 software ecosystem.

### Main components

* **MCU:** STM32G474RET6
* **IDE:** STM32CubeIDE
* **Configuration:** STM32CubeMX
* **Language:** C
* **Motor-control algorithm:** Custom FOC implementation
* **PWM modulation:** SVPWM
* **Position sensor:** MT6816
* **Communication:** Classical CAN

---

# 17. Project Structure

The firmware is organized into application, library, and hardware-specific components.

A typical structure is:

```text
FOC-Driver-V4/
│
├── Core/
│   ├── Inc/
│   └── Src/
│
├── Drivers/
│
├── USB_Device/
│
├── lib/
│   ├── CAN/
│   ├── MT6816/
│   ├── Coefficients/
│   └── ...
│
├── FOC/
│
├── Motor_Control/
│
├── .gitignore
│
└── README.md
```

The exact directory structure may vary as the project develops.

---

# 18. Control Hierarchy

The three control modes are implemented as a hierarchical control system.

```text
                 Position Command
                       │
                       ▼
              ┌─────────────────┐
              │ Position Control│
              └────────┬────────┘
                       │
                  Speed Command
                       │
                       ▼
              ┌─────────────────┐
              │  Speed Control  │
              └────────┬────────┘
                       │
                Current Command
                       │
                       ▼
              ┌─────────────────┐
              │ Current Control │
              └────────┬────────┘
                       │
                       ▼
                      FOC
                       │
                       ▼
                    SVPWM
                       │
                       ▼
                 Motor Inverter
                       │
                       ▼
                    BLDC
```

This structure is particularly useful for robotic applications because a high-level position command can ultimately be translated into precise motor torque through the nested control loops.

---

# 19. Example Application

The driver can be used as an actuator controller in robotic systems.

For example:

```text
Robot Controller
      │
      │ Position Command
      ▼
   CAN Bus
      │
      ▼
FOC Driver Board
      │
      ├── Position Control
      ├── Speed Control
      ├── Current Control
      ├── FOC
      └── SVPWM
      │
      ▼
 BLDC Actuator
```

Multiple boards can be connected to the same CAN network to control multiple joints.

This makes the platform suitable for applications such as:

* Robotic joints
* Humanoid robots
* Mobile robots
* Gimbals
* Servo actuators
* General-purpose BLDC motor control

---

# 20. Engineering Highlights

This project demonstrates practical embedded motor-control development involving both firmware and hardware-level integration.

Key engineering areas include:

### Embedded Systems

* STM32G4 firmware development
* Interrupt-driven real-time control
* Hardware timer configuration
* ADC acquisition
* SPI communication
* CAN communication
* GPIO and fault handling

### Motor Control

* Field-Oriented Control
* Clarke/Park transformations
* Inverse Park transformation
* PI current control
* Cascaded position/speed/current control
* Space Vector PWM

### Hardware Integration

* BLDC three-phase inverter
* Gate-driver interface
* Current sensing
* Magnetic encoder feedback
* DC-bus power stage
* Hardware fault handling

### Communication

* Classical CAN
* Distributed actuator architecture
* Command and status communication
* Integration with higher-level controllers

---

# 21. Project Goals

The main goals of the project are:

1. Develop a custom BLDC motor-control platform.
2. Implement FOC and SVPWM on the STM32G474RET6.
3. Provide multiple closed-loop control modes.
4. Obtain accurate rotor-position feedback using the MT6816.
5. Provide a reliable communication interface through CAN.
6. Create a reusable actuator-control platform for robotic applications.
7. Maintain a modular firmware architecture suitable for further development.

---

# 22. Future Development

Potential future improvements include:

* CAN-FD support
* Advanced motor identification
* Automatic controller parameter tuning
* Field weakening control
* Sensorless FOC
* Improved fault diagnostics
* Motor temperature monitoring
* DC-bus voltage monitoring
* Regenerative braking management
* Higher-level trajectory control
* Integration with ROS 2
* Multi-axis synchronization

---

# 23. Summary

**FOC Driver V4** is a custom embedded motor-control firmware platform based on the **STM32G474RET6**.

The system combines:

```text
STM32G474RET6
      │
      ├── FOC
      │    ├── Clarke Transform
      │    ├── Park Transform
      │    ├── Current Control
      │    └── SVPWM
      │
      ├── Motion Control
      │    ├── Current
      │    ├── Speed
      │    └── Position
      │
      ├── MT6816 Encoder
      │
      └── Classical CAN
```

The resulting platform provides a complete embedded solution for controlling BLDC motors and can serve as a building block for robotic actuators and multi-axis robotic systems.

---

## Author

**Ngoc**

Embedded Systems / Motor Control / Robotics

### Technologies

`C` · `STM32` · `STM32G4` · `FOC` · `SVPWM` · `BLDC` · `CAN` · `SPI` · `PWM` · `ADC` · `MT6816`

---
