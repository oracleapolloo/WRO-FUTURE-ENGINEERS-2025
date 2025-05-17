# WRO-FUTURE-ENGINEERS-2025 - TEAM APOLLO
</p>
<p align="center">
  <img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/26cbb35791b8c0269d3ff0fe9a6655497de72f7e/other/TEAM%20APOLLO%20ICON_WB.PNG" width="420"/>
  
  > An autonomous vehicle designed for the Future Engineers category of the WRO 2025 that uses computer vision and IMU sensors to navigate complex environments and avoid obstacles intelligently.
---
  
## 📁 Content Structure
* `t-photos` – Contains an official team photo and several funny group pics for the vibes 😄  
* `v-photos` – Includes 6 vehicle images: top, bottom, front, back, left, and right  
* `video` – Holds `video.md` with a link to the driving demonstration  
* `schemes` – Contains a PDF file showing the full documentation, wiring, and electromechanical schematics of the robot setup.
* `src` – Full source code for all control components used in the competition  
* `models` – Contains all STL files required for 3D printing the custom components used in the robot. 
* `other` – Contains all other supporting files, including resources used for the creation of this `README.md` and overall documentation. 
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
- [Content Structure](#content-structure)

---

## 🚙 Overview
This self-driving vehicle prototype, developed for the WRO Future Engineers 2025 competition, is a fully autonomous system combining real-time decision-making with a compact and modular hardware layout. It features a PixyCam for object and color-based vision tracking, an MPU6050 (GY-521) IMU for orientation sensing and drift correction, and three HC-SR04 ultrasonic sensors for obstacle detection, wall following, and collision prevention. The robot’s behavior is governed by a combination of PID control, filtered sensor data, and logic-based navigation, allowing it to maintain center alignment and adapt smoothly to changing track environments. Its structure includes prefabricated parts alongside custom 3D-printed mounts, optimized for precise sensor placement and clean wiring. Power is delivered via an 11.1V lithium-ion battery pack using three 18650 cells. The entire system is programmed using Arduino, with modules communicating through PWM, UART, and I²C protocols to coordinate steering, sensors, and motor control. Altogether, this vehicle presents a robust and competition-ready platform that brings together mechanical design, embedded control systems, and smart autonomous behavior.

---

## 🧑‍💻 Team Introduction and Team Information
<table>
  <tr>
    <td align="center" width="33%">
      <img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/d02ef5504a3baa63b37da2d55a1e29b3d79bb3eb/other/pranav%20pic%20wro%20crop.jpg?raw=true" width="100%" style="border-radius: 8px;"><br>
      <strong>PRANAV NAKKEERAN</strong><br>
      <em>Lead Programmer</em><br>
      <sub>Sensor Integration, Logic, Obstacle Avoidance</sub>
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/d02ef5504a3baa63b37da2d55a1e29b3d79bb3eb/other/mifzal%20pic%20wro%20crop.jpg?raw=true" width="100%" style="border-radius: 8px;"><br>
      <strong>MOHAMED MIFZAL MAHAROOF</strong><br>
      <em>Hardware Engineer</em><br>
      <sub>Wiring, Circuit Design, Power Management</sub>
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/d02ef5504a3baa63b37da2d55a1e29b3d79bb3eb/other/aarnav%20pic%20wro%20crop.jpg?raw=true" width="100%" style="border-radius: 8px;"><br>
      <strong>AARNAV BHARGAVA</strong><br>
      <em>Chief Strategist</em><br>
      <sub>Planning, Analysis, Iteration, Strategy Making</sub>
    </td>
  </tr>
</table>

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

### Ultrasonic Sensors (3x HC-SR04)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/3d21aa61965569ea2f02e8448cfda0d1e4c31245/other/HC-SR04.jpeg" width="275" align="right" style="margin-left: 20px;" />

We selected the HC-SR04 because it is:  
> - **Lightweight and low-cost**  
> - **Easy to interface** with Arduino (using digital I/O pins)  
> - **Sufficiently accurate** for our game conditions  

The HC-SR04 is an ultrasonic distance sensor that measures proximity using sound waves. It emits a high-frequency pulse and waits for the echo to return from nearby objects. By measuring the time between the pulse and the echo, it calculates the distance to an obstacle with reasonable accuracy.

In our robot, we used **three HC-SR04 sensors** — positioned at the front, left, and right. The front sensor is used for **collision detection**, ensuring the robot stops before hitting any obstacle. The side sensors are responsible for **wall tracking and edge navigation**, helping the robot stay aligned with track boundaries and make intelligent turning decisions.

In software, we use filtered readings from the sensors to smooth out noise. When the front sensor detects a close obstacle, the robot checks both side distances and turns in the safer direction. This logic also helps determine whether to navigate **clockwise or counterclockwise** during the autonomous laps.
<div style="clear: both;"></div><br>

### 🔄 MG90s Servo Motor (180 Degrees)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/3d21aa61965569ea2f02e8448cfda0d1e4c31245/other/MG90S.JPG" width="275" align="right" style="margin-left: 20px;" />

Why we chose MG90s:
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
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/3d21aa61965569ea2f02e8448cfda0d1e4c31245/other/45503.jpeg" width="275" align="right" style="margin-left: 20px;" />

Why we chose the LEGO EV3 Medium Motor:
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
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/3d21aa61965569ea2f02e8448cfda0d1e4c31245/other/L298N.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose L298N:
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
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/3d21aa61965569ea2f02e8448cfda0d1e4c31245/other/Arduino%20UNO.jpeg" width="275" align="right" style="margin-left: 20px;"/>

Key reasons we chose the Arduino Uno:
>- **Compatibility** with all components used (e.g., HC-SR04, MPU6050, PixyCam)
>- **Stable power management** via Vin (accepts 9–12V from battery pack)
>- **Real-time performance** suitable for obstacle avoidance and PID correction
>- **Robust support for C++ and open-source libraries**

The Arduino Uno is the main microcontroller board used in our robot for processing sensor inputs and controlling all outputs like motors and servos. It features an ATmega328P chip, 14 digital I/O pins (6 PWM-enabled), 6 analog inputs, and USB support for programming.

We selected the Uno for its ease of use, wide compatibility with modules like the L298N, PixyCam, MPU6050, and ultrasonic sensors, and strong community support. It serves as the brain of our system — interpreting sensor data, executing control algorithms, and coordinating all movement and decision logic.

The Uno is powered from the 11.1V battery pack via the **Vin pin**, and shares a **common ground** with all sensors and drivers to maintain consistent signal reference.
<div style="clear: both;"></div><br>

### 📐 GY-521 (MPU6050) – Accelerometer and Gyroscope
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/3d21aa61965569ea2f02e8448cfda0d1e4c31245/other/GY-521.JPG" width="275" align="right" style="margin-left: 20px;" />

Why we chose the MPU6050:
>- **Compact and lightweight** form factor
>- **I2C communication** for easy integration with Arduino
>- **Stable 6-axis motion data** in real-time
>- **Compatible** with PID-based feedback control systems

The GY-521 module features the MPU6050 sensor, which combines a 3-axis accelerometer and a 3-axis gyroscope. It provides precise motion tracking and orientation data. In our robot, this module plays a critical role in maintaining directional accuracy and stability during movement.

We primarily used the MPU6050 for real-time angular drift correction and smooth turning. The gyroscope outputs yaw angle data, which we compare against the desired heading using a PID control loop. Any deviation triggers motor adjustments to bring the robot back on track.

The accelerometer complements the gyro by confirming orientation changes, especially during turns or minor bumps. This dual-sensor setup ensures the robot maintains a straight path and completes clean, accurate rotations.

Its reliability and precision made it ideal for core navigation and turn-based logic across varying field conditions.
<div style="clear: both;"></div><br>

### 🧠 PixyCam (Vision Sensor)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/3d21aa61965569ea2f02e8448cfda0d1e4c31245/other/PixyCam.JPG" width="275" align="right" style="margin-left: 20px;"/>

Key reasons we chose PixyCam:  
> - **Real-time color-based object tracking**  
> - **Fast onboard image processing**  
> - **Reliable detection of multiple objects simultaneously**  
> - **Simple integration** over serial communication (UART/SPI)  

PixyCam is a compact, intelligent vision sensor designed for recognizing and tracking objects based on color signatures. It features a built-in image processor that eliminates the need for external computation, allowing it to detect and follow objects in real time.

In our robot, PixyCam is used during visual challenges to identify and react to colored markers on the field. These color signatures correspond to specific actions like turning, stopping, or path selection. The sensor sends coordinates, color signature IDs, and size data to the Arduino, which then determines the appropriate response.

We chose PixyCam because of its fast processing speed, ease of training, and ability to detect multiple color objects simultaneously. It offers a plug-and-play solution for computer vision in embedded systems, and it integrates cleanly with Arduino via UART or SPI.

This sensor plays a vital role in giving our robot vision-based decision-making, improving its interaction with dynamic environments and challenge-specific elements.
<div style="clear: both;"></div><br>

### 🔋 Power Supply (3x 18650 Battery Pack)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/925b963f555168b780b46e221f96b3fa813c4e4c/other/18650.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose this power system:  
> - **Compact & modular** battery solution  
> - **Ideal voltage** for Arduino & L298N  
> - **Stable performance** during full operation  
> - **Rechargeable & easily replaceable**  
> - **Ensures consistent current delivery**

The robot is powered using three **18650 lithium-ion batteries** connected in series, providing approximately **11.1V** total output. This configuration offers a high energy-to-weight ratio, making it well-suited for mobile robotics.

The Arduino Uno is powered via its **Vin pin**, which supports 9–12V input safely. The same battery pack also powers the L298N motor driver, which controls the LEGO EV3 drive motors. By centralizing power, we ensured reduced cable clutter and better weight distribution.

All components—motors, sensors, Arduino, and drivers—share a **common ground** to maintain stable reference voltage and avoid communication issues.

This configuration ensures sufficient voltage and current to support multiple sensors, servo movement, and motor driving without significant drops or overheating, even during peak loads.
<div style="clear: both;"></div><br>

### 🧱 Custom 3D Printed Components
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/3d21aa61965569ea2f02e8448cfda0d1e4c31245/other/3D-Printed%20Bot.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we 3D printed select parts:  
> - **Precise mounting** for key modules  
> - **Improved center of gravity** for balance  
> - **Modular design** for repairs or upgrades  
> - **Custom fit** for sensors like MPU6050  
> - **Enhanced wiring and airflow**

While our robot’s main frame is built using traditional mounting plates and brackets, we 3D printed specific parts to enhance layout, fit, and function. These include the **gyroscope (MPU6050) base**, and **ultra sonic sensor holders**.

A critical use of 3D printing was to correctly position the MPU6050 at the **robot's center of gravity**, minimizing rotational noise and improving PID accuracy. This helped the gyroscope give more stable yaw values, especially during turns.

We designed these parts in Fusion 360 and printed them using PLA. Their low weight and custom dimensions allowed seamless integration into the chassis without affecting balance or wiring complexity.

This hybrid approach — using both prefabricated and 3D printed parts — gave us more flexibility to prototype and adapt on the fly while keeping the build structurally sound and competition-ready.
<div style="clear: both;"></div><br>

---

## 💻 Software & Libraries

### Tools:
- Arduino IDE
- PixyCam firmware & software

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

## 🎯 Competition Challenges
### 🔗 Challenge Links
- [OPEN CHALLENGE](https://youtu.be/91ZgfnIUOyQ)
- [OBSTACLE CHALLENGE](https://youtu.be/91ZgfnIUOyQ)
### 🎥 Open Challenge Round
[![Watch Open Round](https://img.youtube.com/vi/91ZgfnIUOyQ/0.jpg)](https://youtu.be/91ZgfnIUOyQ)
### 🎥 Obstacle Challenge Round
[![Watch Obstacle Round](https://img.youtube.com/vi/91ZgfnIUOyQ/0.jpg)](https://youtu.be/91ZgfnIUOyQ)

---

## Spoilers
>We stuffed folded paper into weak spots for extra support — not fancy, but it worked. Structural engineering? Nah, just **origami armor** 💀.
---
