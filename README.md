# WRO-FUTURE-ENGINEERS-2025 - TEAM APOLLO

</p>
<p align="center">
  <img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/1f0d68f9e33a2c186d23956248536f89e80ae12d/TEAM%20APOLLO%20ICON_WB.PNG" width="420"/>

  > An autonomous vehicle designed for the Future Engineers category of the WRO 2025 that uses computer vision and IMU sensors to navigate complex environments and avoid obstacles intelligently.

---

## 📌 Table of Contents
- [Overview](#overview)
- [Team Information](#team-information)
- [Project Description](#project-description)
- [Hardware Used](#-hardware-used)
- [Software & Libraries](#software--libraries)
- [Code Explanation](#code-explanation)
- [Challenges & Solutions](#challenges--solutions)
- [Demo / Images](#demo--images)


---

## Overview


This project is a self-driving vehicle prototype built for the WRO Future Engineers 2025 competition. It utilizes a HuskyLens for vision-based obstacle recognition, an MPU6050 IMU for orientation and tilt detection, and ultrasonic sensors for distance measurement. The robot makes intelligent decisions to turn, stop, or accelerate based on sensor data and machine vision feedback.

---

## Team Information

| Name | Role | Responsibility |
|------|------|----------------|
| PRANAV NAKKEERAN | Lead Programmer | Sensor Integration, Logic, Obstacle Avoidance |
| MOHAMED MIFZAL MAHAROOF | Hardware Engineer | Wiring, Circuit Design, Power Management |
| AARNAV BHARGAVA | CHEI~F Strategist |
| Team APOLLO | Dubai, UAE | 🇦🇪 |

---

## 🔍 Project Description

### 💡 Problem Statement
How can a robot dynamically avoid both visible and invisible obstacles in real-time using multi-sensor fusion?

### 🎯 Goals
- Integrate AI vision for object detection.
- Use IMU data to stabilize motion and track orientation.
- Implement ultrasonic-based distance awareness.
- Ensure smooth motor control using L298N.

---

# 🔧 Hardware Used

---

## Ultrasonic Sensors (HC-SR04)

The HC-SR04 ultrasonic sensor is a distance-measuring device that uses sound waves to detect obstacles. It sends out an ultrasonic pulse and waits for the echo to return; by calculating the time taken for the echo, it determines the distance to an object. Three of these sensors were strategically placed on the robot—one at the front, one on the left, and one on the right.

These sensors form the robot’s "eyes" for obstacle detection. The front sensor prevents collisions by detecting direct obstructions, while the side sensors monitor walls and determine when to make left or right turns. We chose this sensor because it is lightweight, inexpensive, and precise enough for our use case.

In our algorithm, distance values are constantly compared against a threshold. If the front sensor detects an obstacle within that range, the robot halts and evaluates side distances to decide which direction is safer to turn. It's also used to determine whether to move clockwise or anticlockwise direction for the game.

---

## 🔄 MG90s Servo Motor (180 Degrees)

The MG90s is a metal-gear micro servo motor with 180 degrees of rotational freedom. We used one MG90s to control the steering mechanism of the robot. It's attached to a set of front wheels via bevel gears, allowing both tires to turn simultaneously for smooth and precise left/right navigation.

Servo motors are ideal for precise angular control, which makes them perfect for steering. We selected the MG90s due to its compact size, high torque, and reliable performance in embedded systems.

The robot uses this motor for direction adjustments. Whenever the algorithm decides to turn, a command is sent to rotate the servo to a specific angle, and then it returns to center after the turn is completed.

---

## ⚙️ LEGO EV3 Medium Servo Motor (45503)

The LEGO EV3 Medium Motor (45503) is a powerful and durable motor capable of variable-speed control. In our build, this motor is responsible for driving the robot forward. It's connected to the rear wheels using a differential gear system that allows the wheels to rotate.

This motor was chosen for its precise speed control and compatibility with our gearing setup. Its torque and RPM balance allow the robot to move smoothly across various terrains. We interfaced it with an Arduino using an L298N motor driver.

The forward movement is controlled via PWM signals, and we modulate these signals based on sensor input to slow down, stop, or accelerate the robot depending on the situation.

---

## ⚡ L298N Motor Driver

The L298N is a dual H-bridge motor driver module that allows control over the speed and direction of DC motors. It acts as the interface between the Arduino and the LEGO EV3 motors, supplying the necessary current and voltage while allowing for directional control via digital signals.

We used the L298N to control the two EV3 motors powering the robot’s movement. It enables us to set motor speed using PWM signals and to change direction using logic inputs. Its built-in heat sink and current-handling capability made it ideal for our setup.

This driver takes PWM from the Arduino and outputs amplified signals to the motors. This setup gives us full control of the robot's motion dynamics.

---

## 📐 GY-521 (MPU6050) – Accelerometer and Gyroscope

The GY-521 module is based on the MPU6050 chip and integrates a 3-axis accelerometer and a 3-axis gyroscope. It provides orientation, tilt, and motion data in real time. In this project, it was critical for maintaining a straight path and enabling accurate turns.

We used the MPU6050 for real-time correction when the robot deviates from its path. The gyroscope data allowed us to detect drift and apply corrective actions using PID (Proportional–Integral–Derivative) control. The accelerometer helped confirm orientation, especially during turns.

By implementing a PID control algorithm, we used the gyro's yaw values to compare current vs. desired heading. The output of the PID was used to adjust motor speeds, ensuring the robot moved straight even on imperfect surfaces or after making a turn.

---

## 🔋 Power Supply

The robot is powered using **three 18650 lithium-ion batteries** connected in series. Each battery provides approximately **3.7V**, giving a total of around **11.1V** output. This voltage is ideal for powering both the Arduino and the motor systems efficiently.

- The **Arduino** is powered through its **Vin pin**, which can safely take 9–12V inputs.
- The **L298N motor driver** is also powered from the same 11.1V supply, which is used to drive the LEGO EV3 motors.
- A **common ground** is shared between all modules (Arduino, sensors, drivers, and motors) to ensure stable and consistent operation.

This configuration provides a reliable, rechargeable power source that is compact and suited for mobile robotic platforms like this one.

---





- HuskyLens (AI Vision Sensor)
- GY-521 MPU6050 (IMU)
- Ultrasonic Sensors (x3)
- LEGO EV3 Medium Motor via L298N Motor Driver
- Servo Motor
- 12V Battery Pack
- Custom 3D Printed Chassis

---

## 💻 Software & Libraries

### Tools:
- Arduino IDE
- HuskyLens firmware & software

### Libraries:
```cpp
#include <Wire.h>
#include <HUSKYLENS.h>
#include <Servo.h>
#include <MPU6050_tockn.h>
