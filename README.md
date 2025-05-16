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
| AARNAV BHARGAVA | CHEIF Strategist |
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

## 🔧 Hardware Used
### Ultrasonic Sensors (HC-SR04)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/16cbb89cfbd837ee0302bcbbc3ca747e88bd4775/HC-SR04.jpeg" width="275" align="right" style="margin-left: 20px;" />
>We selected the HC-SR04 because it is:
>- **Lightweight and low-cost**
>- **Easy to interface** with Arduino (using digital I/O pins)
>- **Sufficiently accurate** for our game conditions

The HC-SR04 is an ultrasonic distance sensor that measures proximity using sound waves. It emits a high-frequency pulse and waits for the echo to return from nearby objects. By measuring the time between the pulse and the echo, it calculates the distance to an obstacle with reasonable accuracy.

In our robot, we used **three HC-SR04 sensors** — positioned at the front, left, and right. The front sensor is used for **collision detection**, ensuring the robot stops before hitting any obstacle. The side sensors are responsible for **wall tracking and edge navigation**, helping the robot stay aligned with track boundaries and make intelligent turning decisions.

In software, we use filtered readings from the sensors to smooth out noise. When the front sensor detects a close obstacle, the robot checks both side distances and turns in the safer direction. This logic also helps determine whether to navigate **clockwise or counterclockwise** during the autonomous laps.
<div style="clear: both;"></div><br>

### 🔄 MG90s Servo Motor (180 Degrees)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/ebe5537ec565bc29764578a813cd18a58e606ea6/MG90S.JPG" width="275" align="right" style="margin-left: 20px;" />
>Why we chose MG90s:
>- **180° rotation** for full turning flexibility
>- **Metal gear construction** for durability
>- **Lightweight** and compact design
>- **Simple PWM control** via Arduino
>- **Quick response time** for rapid steering

The MG90s is a high-torque, metal-gear micro servo motor that supports 180° of rotational movement. In our robot, we used a single MG90s motor to control the steering mechanism, enabling real-time left and right directional changes based on input from the control algorithm.

It is mechanically linked to the front wheels through a bevel gear system, which synchronizes both wheels to turn simultaneously. This allows for precise and responsive steering adjustments, essential for navigating corners, making U-turns, and handling obstacle avoidance routines.

Servo motors like the MG90s are ideal for steering due to their positional accuracy and low-latency response. We selected this model for its compact form factor, high torque-to-size ratio, and durable internal gears that support repeated directional changes during multiple autonomous laps.

This motor ensures smooth, consistent directional control, improving the robot’s agility and responsiveness in both obstacle-rich and open track sections.
<div style="clear: both;"></div><br>

### ⚙️ LEGO EV3 Medium Servo Motor (45503)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/16cbb89cfbd837ee0302bcbbc3ca747e88bd4775/45503.jpeg" width="275" align="right" style="margin-left: 20px;" />
>Why we chose the LEGO EV3 Medium Motor:
>- **High torque** with compact design
>- **Precise speed control** using encoders
>- **Compatible** with our differential drivetrain
>- **Reliable performance** under load
>- **Robust housing** and long-term durability

The LEGO EV3 Medium Motor (45503) is a compact yet powerful servo motor capable of precise speed and directional control. We used this motor to drive the robot’s rear wheels using a differential gear system, enabling smooth and efficient linear movement.

This motor provides the necessary torque and RPM for consistent motion across various field surfaces. Its internal rotary encoder allows for accurate speed regulation, which is essential when executing PID-controlled driving strategies or navigating tight track corners.

We interfaced the EV3 motor with the Arduino via the L298N motor driver, allowing full control using PWM signals. The Arduino adjusts these signals in real time based on sensor feedback from the ultrasonic sensors and gyroscope.

Its combination of control precision, power efficiency, and mechanical robustness made it the ideal choice for driving the robot during autonomous navigation and obstacle-avoidance tasks.
<div style="clear: both;"></div><br>

### ⚡ L298N Motor Driver
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/16cbb89cfbd837ee0302bcbbc3ca747e88bd4775/L298N.jpg" width="275" align="right" style="margin-left: 20px;" />
>Why we chose L298N:
>- **Dual H-bridge** control for 2 motors
>- **PWM speed control** via Arduino
>- **Reversing logic** with digital signals
>- **Integrated heat sink** for thermal protection
>- **Readily available and cost-effective**

The L298N is a dual H-bridge motor driver that allows control over both the speed and direction of two DC motors independently. It serves as the main interface between the Arduino and the LEGO EV3 motors used in our robot, enabling precise control using standard logic-level signals.

In our design, the L298N receives PWM signals from the Arduino to control the EV3 motor speeds and digital HIGH/LOW logic to reverse direction. This flexibility allows us to dynamically adjust movement, perform turns, and apply PID-based corrections in real time.

Its built-in heat sink and ability to handle high current made it ideal for our application, especially with heavy-duty motors like the EV3 series. It supports up to 2A per channel and is robust enough for continuous driving.

The L298N gave us the flexibility and power needed to drive the robot consistently across different movement modes.
<div style="clear: both;"></div><br>

### 🧠 Arduino Uno (ATmega328P)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/16cbb89cfbd837ee0302bcbbc3ca747e88bd4775/Arduino%20UNO.jpeg" width="275" align="right" style="margin-left: 20px;"/>
>Key reasons we chose the Arduino Uno:
>- **Compatibility** with all components used (e.g., HC-SR04, MPU6050, HuskyLens)
>- **Stable power management** via Vin (accepts 9–12V from battery pack)
>- **Real-time performance** suitable for obstacle avoidance and PID correction
>- **Robust support for C++ and open-source libraries**

The Arduino Uno is the main microcontroller board used in our robot for processing sensor inputs and controlling all outputs like motors and servos. It features an ATmega328P chip, 14 digital I/O pins (6 PWM-enabled), 6 analog inputs, and USB support for programming.

We selected the Uno for its ease of use, wide compatibility with modules like the L298N, HuskyLens, MPU6050, and ultrasonic sensors, and strong community support. It serves as the brain of our system — interpreting sensor data, executing control algorithms, and coordinating all movement and decision logic.

The Uno is powered from the 11.1V battery pack via the **Vin pin**, and shares a **common ground** with all sensors and drivers to maintain consistent signal reference.
<div style="clear: both;"></div><br>

### 📐 GY-521 (MPU6050) – Accelerometer and Gyroscope
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/ebe5537ec565bc29764578a813cd18a58e606ea6/GY-521.JPG" width="275" align="right" style="margin-left: 20px;" />
>Why we chose the MPU6050:
>- **Compact and lightweight** form factor
>- **I2C communication** for easy integration with Arduino
>- **Stable 6-axis motion data** in real-time
>- **Compatible** with PID-based feedback control systems

The GY-521 module features the MPU6050 sensor, which combines a 3-axis accelerometer and a 3-axis gyroscope. It provides precise motion tracking and orientation data. In our robot, this module plays a critical role in maintaining directional accuracy and stability during movement.

We primarily used the MPU6050 for real-time angular drift correction and smooth turning. The gyroscope outputs yaw angle data, which we compare against the desired heading using a PID control loop. Any deviation triggers motor adjustments to bring the robot back on track.

The accelerometer complements the gyro by confirming orientation changes, especially during turns or minor bumps. This dual-sensor setup ensures the robot maintains a straight path and completes clean, accurate rotations.

Its reliability and precision made it ideal for core navigation and turn-based logic across varying field conditions.
<div style="clear: both;"></div><br>

### 🧠 HuskyLens (AI Vision Sensor)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/ebe5537ec565bc29764578a813cd18a58e606ea6/HUSKYLENS.jpg" width="275" align="right" style="margin-left: 20px;"/>
>Key reasons we chose HuskyLens:
>- **One-click training** for multiple vision modes
>- **Live visual feedback** on the sensor’s screen
>- **Fast onboard AI** (no internet/cloud required)
>- **Easy integration** over serial communication (UART)

HuskyLens is an AI-powered vision sensor capable of object detection, face recognition, color recognition, line following, and tag scanning. It features onboard machine learning with a built-in display, enabling training and testing without a PC.

In our robot, HuskyLens is used for real-time object detection and line following during visual challenges. We trained it to recognize specific object shapes and colored markers placed on the field. Based on visual input, the Arduino adjusts movement, such as switching direction or triggering obstacle routines.

HuskyLens was selected for its simplicity, standalone learning capability, and ability to integrate seamlessly with Arduino via UART. The onboard interface allowed us to re-train and test directly in the field with no coding changes.

This sensor acts as the robot’s "vision brain" for intelligent decisions based on visual patterns and object proximity.
<div style="clear: both;"></div><br>

### 🔋 Power Supply (3x 18650 Battery Pack)
The robot is powered using **three 18650 lithium-ion batteries** connected in series, giving a total of approximately **11.1V**.

- The **Arduino** is powered via its **Vin pin**, accepting 9–12V input.
- The **L298N** draws power from the same source and drives the EV3 motors.
- A **common ground** is maintained across all components for signal stability.

This setup provides enough voltage and current to power all modules simultaneously without voltage drops or overheating issues.
<div style="clear: both;"></div><br>

### 🧱 Custom 3D Printed Chassis
The robot’s body was custom-designed using Fusion 360 and 3D printed using PLA material. It features compartments and mounting slots for all components including the Arduino, L298N, sensors, and battery pack.

Designing the chassis in-house allowed us to create a compact, structurally balanced frame optimized for center of gravity and accessibility during repairs. This helped reduce weight and simplified wiring.

The body was printed in multiple parts and assembled using screws, with space allocated for airflow, cable routing, and sensor field-of-view.
<div style="clear: both;"></div><br>
---

## 💻 Software & Libraries

### Tools:
- Arduino IDE
- HuskyLens firmware & software

### getFilteredDistance():
#### Reliable Sensor Reading
```cpp
float getFilteredDistance(NewPing &sonar) {
  const int samples = 3; // Sample count
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    float dist = sonar.ping_cm();
    if (dist == 0) dist = MAX_DISTANCE; // Handle no response
    sum += dist;
    delay(5); // Allow sensor to settle
  }
  return sum / samples;
}
```
>	•	Purpose: Averages multiple ultrasonic readings to reduce noise.<br>
> •	Impact: Provides stable and trustworthy data used in wall-following and edge-detection decisions.<br>
> •	Why filter? Ultrasonic sensors are prone to false or noisy readings. Using 3 samples reduces random fluctuations.

### performTurn():
#### Precision Turns Using Gyroscope
```cpp
void performTurn(bool clockwise) {
  mpu.update();
  float startAngle = mpu.getAngleZ();
  float turnTarget = startAngle + (clockwise ? -88 : 88); // Turn ±88 degrees
  analogWrite(MOTOR_ENB, 0);
  steeringServo.write(clockwise ? SERVO_MAX_RIGHT : SERVO_MAX_LEFT);

  while (abs(mpu.getAngleZ() - turnTarget) > 2) {
    mpu.update();
    analogWrite(MOTOR_ENB, SLOW_SPEED);
  }

  analogWrite(MOTOR_ENB, 0);
  steeringServo.write(SERVO_CENTER);
  targetAngle = turnTarget;
}
```
>	•	Purpose: Turns the vehicle accurately by ~90°.
> •	Decision Factor: Chooses turn direction based on wall distances and uses MPU6050 to measure turn angle.
> •	Precision Control: Ensures consistent turns even after multiple laps (essential for edge counting).

### Main Control Logic:
#### loop() Core Navigation & Correction
```cpp
float gyroError = targetAngle - currentAngle;
int servoAngle = SERVO_CENTER;
if (directionSet && !isWaiting) {
  float innerDist = isClockwise ? leftDist : rightDist;
  float outerDist = isClockwise ? rightDist : leftDist;
  float wallError = TARGET_WALL_DISTANCE - innerDist;
  float outerError = outerDist < OUTER_WALL_THRESHOLD ? OUTER_WALL_THRESHOLD - outerDist : 0;
  
  servoAngle += (int)(KP * gyroError + KW * wallError + KO * outerError);
} else {
  servoAngle += (int)(KP * gyroError);
}
servoAngle = constrain(servoAngle, 60, 120);
steeringServo.write(servoAngle);
```
> • Multifactor Decision: Blends three correction mechanisms:<br>
>> &nbsp;&nbsp;&nbsp;&nbsp;1. Gyro (MPU) to correct angular drift.<br>
>> &nbsp;&nbsp;&nbsp;&nbsp;2. Inner Wall distance to maintain center path.<br>
>> &nbsp;&nbsp;&nbsp;&nbsp;3. Outer Wall avoidance when too close to obstacles.<br>

> • Dynamic Steering: Adjusts the servo in real-time using a weighted formula.<br>
> • Smart Navigation: Enables adaptive behavior depending on lap direction (`isClockwise`) and surrounding environment.

### Turn Initiation Based on Sensor Input:
```cpp
if (!directionSet) {
  if (rightDist > SIDE_TURN_DISTANCE && rightDist < MAX_DISTANCE) {
    isClockwise = true;
    directionSet = true;
    edgeCount++;
    performTurn(isClockwise);
    isWaiting = true;
    waitStartTime = millis();
  } else if (leftDist > SIDE_TURN_DISTANCE && leftDist < MAX_DISTANCE) {
    isClockwise = false;
    directionSet = true;
    edgeCount++;
    performTurn(isClockwise);
    isWaiting = true;
    waitStartTime = millis();
  }
}
```
> •	Decision Maker: This is where the robot decides its lap direction based on which side wall disappears first.<br>
> •	Sets Behavior: Once set, isClockwise changes all future logic.<br>
> •	Edge Detection: First increment of edgeCount, starting lap tracking.

### Edge Detection & Lap Completion:
```cpp
float outerDist = isClockwise ? rightDist : leftDist;
if (outerDist > SIDE_TURN_DISTANCE && outerDist < MAX_DISTANCE) {
  edgeCount++;
  if (edgeCount >= TOTAL_EDGES) {
    performTurn(isClockwise); // Final turn
    delay(1000); // Move forward
    analogWrite(MOTOR_ENB, 0); // Stop
    isRunning = false;
    while (1); // Halt forever
  }
  performTurn(isClockwise);
  isWaiting = true;
  waitStartTime = millis();
}
```
>	•	Edge Counting: Core mechanism to track progress across laps and corners.<br>
> •	Laps Tracking: Uses a fixed TOTAL_EDGES = 13 to know when to stop.<br>
> •	Autonomous Exit: Robot completes a routine and safely stops.
---
