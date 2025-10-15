# PID-Motor-Position-Control-with-L298N


## Overview
This project implements **closed-loop PID control** for a **GA12 N20 DC motor** using an **L298N motor driver** and **Arduino Uno**.  
The system reads the motor’s position using a **quadrature encoder**, calculates the error between the desired and current position, and applies PID control to reach precise angles.

---

## Components
- Arduino Uno  
- L298N Motor Driver  
- GA12 N20 Geared DC Motor with Encoder  
- 6V Power Supply  
- Jumper Wires  
- Breadboard / Chassis (optional)

---

## Features
- Real-time PID control with **tunable Kp, Ki, Kd**  
- Encoder feedback for accurate position tracking  
- Serial commands for easy angle and PID tuning  
- Auto stop when target position is reached  

---

## Serial Commands
| Command | Example | Description |
|---------|---------|-------------|
| `g<angle>` | `g90` | Move motor to 90° |
| `p<value>` | `p8.0` | Set proportional gain |
| `i<value>` | `i20` | Set integral gain |
| `d<value>` | `d0.8` | Set derivative gain |
| `r` | - | Reset position to 0° |
| `s` | - | Show current PID status |

---

## Circuit Connections

| Component | Arduino Pin |
|-----------|-------------|
| ENA (L298N) | D6 |
| IN1 | D7 |
| IN2 | D8 |
| Encoder A | D2 |
| Encoder B | D3 |

> **Note:** Connect L298N GND to Arduino GND and supply motor with 6V power.

---

## How It Works
1. Encoder counts are converted to angles:Angle = (EncoderCount / 700) * 360°
2. PID controller calculates output voltage: Output = Kp * error + Ki * integral + Kd * derivative
3. Output voltage is converted to PWM and applied to the L298N to drive the motor.  
4. Motor stops automatically when within the specified tolerance of the target angle.

---

## 📊 Example Output

Target: 90.0° | Current: 85.3° | Error: 4.7° | PWM: 180 | STATUS: MOVING
Target: 90.0° | Current: 90.1° | Error: 0.1° | PWM: 25 | STATUS: AT POSITION


