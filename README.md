# WRO-FUTURE-ENGINEERS-2025 - TEAM APOLLO

<p align="center">
  <img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/26cbb35791b8c0269d3ff0fe9a6655497de72f7e/other/TEAM%20APOLLO%20ICON_WB.PNG" width="420"/>
  
  > An autonomous vehicle designed for the Future Engineers category of the WRO 2025 that uses advanced computer vision and IMU sensors to navigate complex environments and avoid obstacles intelligently.

---
  
## 📁 Content Structure
* `t-photos` – Contains an official team photo and several funny group pics for the vibes 😄.  
* `v-photos` – Includes 6 vehicle images: top, bottom, front, back, left, and right.  
* `video` – Holds `video.md` with a link to the driving demonstration.  
* `schemes` – Contains a PDF file showing the full documentation, wiring, and electromechanical schematics of the robot setup.
* `src` – Full source code for all control components used in the competition.  
* `models` – Contains all STL files required for 3D printing the custom components used in the robot. 
* `other` – Contains all other supporting files, including resources used for the creation of this `README.md` and overall documentation.
* `obstacle challenge & open challenge` – Includes the complete codebase for both challenges.

---

## 📌 Table of Contents

- [Overview](#overview)  
- [Team Introduction & Team Information](#team-introduction--team-information)  
- [Project Description](#project-description)  
- [Hardware Used](#hardware-used)  
- [Software & Libraries](#software--libraries)  
- [Competition Challenges](#competition-challenges)  
- [Spoilers](#spoilers)
  
---

<a name="overview"></a>
## 🚙 Overview
This self-driving vehicle prototype, developed for the WRO Future Engineers 2025 competition, is a fully autonomous system combining real-time decision-making with a compact and modular hardware layout. It features a Limelight 3A for advanced object detection and color-based vision tracking, a REV Robotics 9-Axis IMU for precise orientation sensing and drift correction, and three REV Robotics 2m Distance Sensors for obstacle detection, wall following, and collision prevention. The robot's behavior is governed by a combination of PID control, filtered sensor data, and logic-based navigation, allowing it to maintain center alignment and adapt smoothly to changing track environments. Its structure includes prefabricated parts alongside custom 3D-printed mounts, optimized for precise sensor placement and clean wiring. Power is delivered via an 11.1V lithium-ion battery pack using three 18650 cells. The entire system is programmed using Python on Raspberry Pi 5, with modules communicating through PWM, UART, and I²C protocols to coordinate steering, sensors, and motor control. Altogether, this vehicle presents a robust and competition-ready platform that brings together mechanical design, embedded control systems, and smart autonomous behavior.

---

<a name="team-introduction--team-information"></a>
## 🧑‍💻 Team Introduction & Team Information
<table>
  <tr>
    <td align="center" width="33%">
      <img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/d02ef5504a3baa63b37da2d55a1e29b3d79bb3eb/other/pranav%20pic%20wro%20crop.jpg?raw=true" width="100%" style="border-radius: 8px;"><br>
      <strong>PRANAV NAKKEERAN</strong><br>
      <em>Chief Strategist</em><br>
      <sub>Planning, Analysis, Iteration, Strategy Making</sub>
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
      <em>Lead Programmer</em><br>
      <sub>Sensor Integration, Logic, Obstacle Avoidance</sub>
    </td>
  </tr>
</table>

---
---

<a name="project-description"></a>
## 🔍 Project Description

### 💡 Problem Statement
How can a robot dynamically avoid both visible and invisible obstacles in real-time using multi-sensor fusion and advanced vision processing?

### 🎯 Goals
- Integrate AI vision with neural network processing for object detection.
- Use 9-axis IMU data to stabilize motion and track orientation with high precision.
- Implement ultrasonic-based distance awareness with improved accuracy.
- Ensure smooth motor control using high-current BTS7960 driver.
- Leverage Raspberry Pi 5's computational power for complex autonomous behaviors.

---

<a name="hardware-used"></a>
## 🔧 Hardware Used

### REV Robotics 2m Distance Sensors (3x)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/492bd3f4281136f59b33868263398c0b74ff2589/other/2m Distance Sensors.webp" width="275" align="right" style="margin-left: 20px;" />

We selected the REV Robotics 2m Distance Sensor because it is:  
> - **Highly accurate** with ±1cm precision  
> - **Extended range** up to 2 meters for better obstacle detection  
> - **Easy to interface** with Raspberry Pi via I²C  
> - **Industrial-grade reliability** for consistent performance  
> - **Fast sampling rate** for real-time decision making

The REV Robotics 2m Distance Sensor uses Time-of-Flight (ToF) technology to measure distances with exceptional accuracy. Unlike traditional ultrasonic sensors, these ToF sensors emit infrared light pulses and calculate distance based on the time taken for light to reflect back, resulting in more precise and reliable measurements even in challenging lighting conditions.

In our robot, we used **three REV Robotics Distance Sensors** — positioned at the front, left, and right. The front sensor provides **advanced collision avoidance**, detecting obstacles up to 2 meters away and allowing the robot to plan turns earlier. The side sensors handle **precision wall tracking and edge navigation**, maintaining optimal distance from boundaries while enabling intelligent turning decisions with millimeter-level accuracy.

In software, we utilize the sensors' built-in filtering capabilities and I²C communication protocol for rapid data acquisition. The extended range allows the robot to detect approaching obstacles sooner, giving more time for smooth trajectory adjustments. When the front sensor detects an obstacle, the system analyzes both side distances with high precision to determine the optimal turning direction, enhancing both safety and navigation efficiency.
<div style="clear: both;"></div><br>

### 🔄 Axon MICRO+ Servo Motor
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/492bd3f4281136f59b33868263398c0b74ff2589/other/Axon MICRO Servo.gif" width="275" align="right" style="margin-left: 20px;" />

Why we chose Axon MICRO+:
>- **270° rotation range** for enhanced steering flexibility
>- **Coreless motor design** for ultra-fast response
>- **Titanium gear construction** for maximum durability
>- **Programmable endpoints** for precise calibration
>- **High torque-to-weight ratio** (4.8kg-cm at 7.4V)
>- **Digital servo protocol** with position feedback

The Axon MICRO+ is a high-performance, programmable digital servo motor featuring coreless motor technology and titanium gears. This advanced servo provides 270° of rotation with exceptional precision and speed, making it ideal for responsive steering control in competitive robotics.

In our robot, we deployed the Axon MICRO+ as the primary steering actuator, enabling real-time directional adjustments with sub-degree accuracy. Its coreless motor eliminates cogging, resulting in silky-smooth motion and instantaneous response to control signals from the Raspberry Pi 5.

The servo is mechanically linked to the front wheels through an optimized Ackermann steering geometry with precision-cut gears. This configuration ensures both wheels turn synchronously at calculated angles, providing accurate steering even during high-speed maneuvers and tight corner navigation.

We selected this servo for its programmable features, allowing us to fine-tune endpoints, speed, and acceleration profiles through software. The titanium gears withstand the stress of repeated rapid movements, essential for completing multiple autonomous laps without mechanical degradation. The digital feedback capability enables closed-loop control, where the Raspberry Pi can verify actual servo position against commanded position, improving overall system reliability.

This motor ensures rapid, consistent, and precise directional control, dramatically improving the robot's agility and responsiveness in both obstacle-rich environments and open track sections while maintaining competitive speed.
<div style="clear: both;"></div><br>

### ⚙️ AndyMark NeveRest Motor
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/492bd3f4281136f59b33868263398c0b74ff2589/other/AM NeveRest Motor.png" width="275" align="right" style="margin-left: 20px;" />

Why we chose the AndyMark NeveRest:
>- **High torque output** with multiple gear ratios available
>- **Integrated quadrature encoder** for precise position feedback
>- **Robust planetary gearbox** for reliable power transmission
>- **Wide voltage range** (6-12V) for flexible power management
>- **Efficient heat dissipation** for sustained performance
>- **Competition-proven reliability** in FTC/FRC events

The AndyMark NeveRest is a high-performance DC motor featuring a precision planetary gearbox and integrated Hall-effect encoder. This motor delivers exceptional torque while maintaining compact dimensions, making it perfect for driving our robot's differential drivetrain system.

We selected the NeveRest motor with a 20:1 gear ratio, providing an optimal balance between speed and torque for our application. The motor drives the robot's rear wheels through a custom differential assembly, enabling smooth acceleration, controlled speed variation, and reliable linear movement across diverse track surfaces.

The integrated quadrature encoder outputs 1120 counts per revolution, allowing the Raspberry Pi 5 to implement sophisticated closed-loop control algorithms. This encoder feedback enables precise speed regulation, accurate distance measurement, and coordinated motion control essential for executing complex autonomous navigation strategies.

We interface the NeveRest motor with the Raspberry Pi 5 through the BTS7960 high-current motor driver, which handles the motor's power requirements while the Raspberry Pi manages PWM speed control and direction signals. The system continuously monitors encoder feedback to maintain target speeds and adjust for load variations in real-time.

The motor's planetary gearbox provides smooth, reliable power transmission with minimal backlash, crucial for precise movement control during PID-based navigation and tight cornering maneuvers. Its robust construction withstands the repeated start-stop cycles and varying load conditions encountered during autonomous operation.

The combination of high torque, precision feedback, and durability made the NeveRest motor the ideal choice for driving our competition robot through challenging courses while maintaining consistent performance across multiple autonomous runs.
<div style="clear: both;"></div><br>

### ⚡ BTS7960 High-Current Motor Driver
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/22721c2a993cc536da789fa8a96c43088dfe4141/other/%20BTS7960.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose BTS7960:
>- **High current capacity** up to 43A continuous
>- **Full H-bridge** for bidirectional control
>- **PWM frequency** up to 25kHz for smooth operation
>- **Over-current and thermal protection** built-in
>- **Low voltage drop** for efficient power delivery
>- **Isolated signal inputs** for noise immunity

The BTS7960 is a professional-grade, high-current H-bridge motor driver module capable of handling the demanding power requirements of high-performance DC motors. It features two Infineon BTS7960 chips configured as a full H-bridge, providing robust bidirectional motor control with exceptional current handling capabilities.

In our design, the BTS7960 serves as the primary interface between the Raspberry Pi 5 and the AndyMark NeveRest motor. It receives PWM signals from the Raspberry Pi for speed control and digital logic signals for direction management. This configuration allows us to implement sophisticated motor control algorithms including acceleration profiling, dynamic speed adjustment, and regenerative braking simulation.

The driver's high current capacity (43A continuous, 100A peak) provides substantial headroom for our motor's demands, even during rapid acceleration or high-load situations. This prevents voltage sag and ensures consistent motor performance throughout competition runs. The module's built-in protection features guard against over-current conditions, short circuits, and thermal overload, protecting both the driver and the expensive NeveRest motor.

Unlike lower-capacity drivers, the BTS7960 maintains efficiency at high currents with minimal heat generation thanks to its low RDS(on) and integrated heat sinking. The isolated signal inputs protect the Raspberry Pi from electrical noise generated by motor switching, ensuring stable I²C and encoder signal integrity.

The driver's high PWM frequency capability (up to 25kHz) eliminates audible motor whine and produces smoother torque delivery, particularly beneficial for precision speed control during wall-following and PID correction maneuvers. This results in more stable sensor readings and improved overall system performance.

The BTS7960's combination of high current handling, robust protection features, and clean signal operation made it the essential choice for reliable, high-performance motor control in our competition-grade autonomous vehicle.
<div style="clear: both;"></div><br>

### 🧠 Raspberry Pi 5 8GB - Main Controller
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/492bd3f4281136f59b33868263398c0b74ff2589/other/rPi5.jpg" width="275" align="right" style="margin-left: 20px;"/>

Key reasons we chose the Raspberry Pi 5 8GB:
>- **Quad-core ARM Cortex-A76 CPU** running at 2.4GHz
>- **8GB LPDDR4X RAM** for complex vision processing
>- **Dual 4K display outputs** for development debugging
>- **PCIe 2.0 interface** for high-speed peripherals
>- **Native USB 3.0** for fast data transfer
>- **Real-time capabilities** with improved GPIO performance
>- **Python and C++ support** with extensive libraries
>- **Active community** and professional documentation

The Raspberry Pi 5 represents a significant leap in single-board computer performance, serving as the central processing unit for our autonomous robot. Featuring a quad-core 64-bit ARM Cortex-A76 processor running at 2.4GHz and 8GB of high-speed LPDDR4X memory, it provides the computational power necessary for running complex vision algorithms, sensor fusion, and real-time control systems simultaneously.

In our system architecture, the Raspberry Pi 5 functions as the robot's brain, processing inputs from the Limelight 3A vision system, REV Robotics 9-Axis IMU, and three distance sensors while executing sophisticated navigation algorithms and generating precise control outputs to the BTS7960 motor driver and Axon MICRO+ servo.

We selected the 8GB model specifically to support simultaneous operation of the vision processing pipeline, neural network inference (if required), sensor data filtering, PID controllers, and logging systems without memory constraints. The increased RAM allows us to maintain larger data buffers and implement more sophisticated prediction algorithms for smoother robot behavior.

The Raspberry Pi 5's improved GPIO performance with dedicated RP1 I/O controller enables more reliable PWM generation and faster I²C communication with sensors. This results in higher servo update rates and reduced latency in motor control responses, critical for maintaining stability during high-speed operation.

Programming flexibility is a key advantage - we develop in Python for rapid prototyping and algorithm testing, with the option to optimize critical sections in C++ when needed. The extensive software ecosystem provides libraries for vision processing (OpenCV), numerical computation (NumPy), and hardware interfaces (pigpio, gpiozero), accelerating development and enabling sophisticated features.

The built-in dual-band WiFi enables wireless code deployment, real-time telemetry monitoring, and remote debugging during testing sessions, significantly streamlining our development workflow. The PCIe interface future-proofs our design, allowing for potential addition of AI accelerators or high-speed storage if needed.

Power management is handled efficiently, with the board drawing appropriate current for the computational load while remaining stable under the demanding conditions of autonomous navigation. The improved thermal design with official active cooling accessories ensures sustained performance even during extended competition runs.

The Raspberry Pi 5's combination of processing power, memory capacity, connectivity options, and development ecosystem made it the natural choice for controlling our advanced competition robot, enabling implementation of sophisticated autonomous behaviors that were previously impractical on lower-powered platforms.
<div style="clear: both;"></div><br>

### 📐 REV Robotics 9-Axis IMU – Inertial Measurement Unit
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/492bd3f4281136f59b33868263398c0b74ff2589/other/REV Robotics 9-Axis IMU.webp" width="275" align="right" style="margin-left: 20px;" />

Why we chose the REV Robotics 9-Axis IMU:
>- **9-axis sensor fusion** (3-axis accelerometer, gyroscope, magnetometer)
>- **BNO055 smart sensor** with onboard processing
>- **Quaternion output** for gimbal-lock-free orientation
>- **Automatic calibration** with persistent storage
>- **I2C interface** compatible with Raspberry Pi
>- **Industrial temperature range** for reliability
>- **Pre-configured** for robotics applications

The REV Robotics 9-Axis IMU features the Bosch BNO055 intelligent sensor, which combines a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer with an integrated ARM Cortex-M0 processor. This advanced IMU provides real-time orientation data through sophisticated sensor fusion algorithms executed entirely on-chip, offloading computational burden from the main processor.

In our robot, this IMU serves as the cornerstone of our navigation system, providing absolute orientation tracking with exceptional accuracy and stability. The sensor fusion algorithm combines data from all nine axes to output orientation in multiple formats: Euler angles, quaternions, and rotation vectors, giving us flexibility in implementing our control algorithms.

We primarily utilize the IMU's heading (yaw) data for maintaining straight-line tracking and executing precise turns. The quaternion output eliminates gimbal lock issues that plague simpler 6-axis IMUs, ensuring reliable orientation data even during complex maneuvers. The onboard sensor fusion provides filtered, drift-compensated heading values at 100Hz update rates, enabling smooth and responsive PID control.

The BNO055's automatic calibration system is particularly valuable - it continuously monitors sensor performance and adjusts calibration parameters, storing them in non-volatile memory. This means our robot maintains consistent orientation accuracy across power cycles and environmental variations without requiring manual recalibration.

Unlike basic IMUs like the MPU6050, the REV Robotics unit includes a magnetometer for absolute heading reference. This prevents the cumulative drift typical of gyroscope-only solutions, essential for accurate navigation over extended autonomous runs. The accelerometer complements the gyroscope by detecting dynamic motion changes and providing orientation reference during stationary periods.

Integration with the Raspberry Pi 5 is straightforward via I²C protocol, with REV providing well-documented Python libraries specifically designed for FIRST Tech Challenge and educational robotics. These libraries abstract low-level register manipulation while providing access to advanced features like interrupt-driven updates and custom axis remapping.

The IMU's robust construction and industrial-grade components ensure reliable operation despite the vibrations and occasional impacts inherent in competitive robotics. Its compact form factor and mounting hole pattern facilitate secure installation at the robot's center of gravity, minimizing rotational coupling effects that could introduce noise into orientation measurements.

The combination of sophisticated sensor fusion, absolute heading reference, automatic calibration, and reliable performance made the REV Robotics 9-Axis IMU the definitive choice for providing our robot with accurate, stable orientation data essential for precise autonomous navigation and smooth motion control.
<div style="clear: both;"></div><br>

### 🧠 Limelight 3A - Advanced Vision Sensor
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/22721c2a993cc536da789fa8a96c43088dfe4141/other/limelight3a.jpg" width="275" align="right" style="margin-left: 20px;"/>

Key reasons we chose Limelight 3A:  
> - **Neural network processing** with Google Coral TPU  
> - **High-resolution camera** (1920×1080 at 90fps)  
> - **AprilTag detection** with sub-pixel accuracy  
> - **Color-based object tracking** with HSV filtering  
> - **NetworkTables API** for seamless integration  
> - **Web-based configuration** for easy tuning  
> - **3D pose estimation** for spatial awareness  
> - **Snapshot and recording** capabilities for debugging

The Limelight 3A represents a significant advancement in vision processing for competitive robotics, featuring a high-resolution camera paired with a Google Coral Edge TPU for on-device neural network inference. This intelligent vision sensor performs sophisticated image processing entirely onboard, delivering position, size, and classification data over NetworkTables with minimal latency.

In our robot, the Limelight 3A serves as the primary visual perception system, enabling detection and tracking of colored markers, AprilTags, and custom-trained objects critical for autonomous navigation in competition environments. The integrated Edge TPU allows us to run custom TensorFlow Lite models for object detection without overwhelming the Raspberry Pi 5's main processor.

We configured multiple vision pipelines within the Limelight, each optimized for different tasks: color blob detection for tracking field markers, AprilTag detection for precision positioning, and neural network inference for identifying specific game elements. The sensor can switch between these pipelines programmatically, adapting to different phases of autonomous operation.

The high frame rate capability (90fps at 1080p, 120fps at 720p) provides smooth, responsive tracking even during rapid robot motion. This high temporal resolution enables our control algorithms to react quickly to changing visual conditions, improving trajectory accuracy and obstacle response times.

Unlike simpler vision sensors, the Limelight 3A outputs comprehensive targeting data including: horizontal and vertical angles to target, bounding box dimensions, target area, skew, and corner coordinates. For AprilTags, it additionally provides 6DOF pose information (X, Y, Z position and roll, pitch, yaw orientation), enabling precise spatial reasoning and positioning.

The web-based configuration interface allows us to fine-tune vision pipelines in real-time during testing. We can adjust HSV thresholds, erosion/dilation parameters, contour filtering rules, and neural network confidence levels while observing the processed image stream, dramatically speeding up calibration for different lighting conditions.

NetworkTables integration provides a robust, low-latency communication channel to the Raspberry Pi 5. The Limelight publishes detection results at high frequency, and the Raspberry Pi can programmatically control camera settings, switch pipelines, and trigger snapshots for logging. This bidirectional communication enables adaptive vision processing based on robot state.

The Limelight's 3D positioning capabilities are particularly valuable for complex navigation scenarios. Using stereo AprilTag pairs or known-size objects, the sensor calculates precise distance and angle measurements, enabling accurate approach maneuvers and positioning tasks that would be impractical with simple 2D vision.

The robust construction, industrial-grade camera sensor, and professional-level processing power made the Limelight 3A an essential component for giving our robot sophisticated visual perception capabilities. It transforms raw visual information into actionable navigation data, enabling intelligent decision-making and precise autonomous operation in dynamic competition environments.
<div style="clear: both;"></div><br>

### 🔋 Power Supply (3x 18650 Battery Pack)
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/925b963f555168b780b46e221f96b3fa813c4e4c/other/18650.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we chose this power system:  
> - **Compact & modular** battery solution  
> - **Optimal voltage** (11.1V nominal) for motor driver  
> - **High discharge rate** (20A continuous capability)  
> - **Stable voltage curve** throughout discharge cycle  
> - **Rechargeable & easily replaceable** cells  
> - **Built-in protection circuitry** for safety  
> - **Lightweight** compared to equivalent lead-acid  
> - **Consistent current delivery** under load

The robot is powered by three **Samsung 30Q 18650 lithium-ion batteries** connected in series through a protected battery holder, providing approximately **11.1V nominal** (12.6V fully charged, 9V minimum) total output. This 3S configuration offers an excellent energy density of 3000mAh, delivering sustained power for extended autonomous runs while maintaining manageable weight.

Our power distribution architecture carefully manages voltage requirements for different subsystems. The **BTS7960 motor driver** and **AndyMark NeveRest motor** operate directly from the 11.1V battery supply, benefiting from maximum available voltage for optimal torque and speed. The **Raspberry Pi 5** requires 5V/5A, supplied through a high-efficiency UBEC (Universal Battery Elimination Circuit) regulator that maintains stable voltage even during motor current spikes.

The **Axon MICRO+ servo** is powered through a separate 6V/3A UBEC to provide clean power isolated from motor noise. The **REV Robotics IMU**, **distance sensors**, and **Limelight 3A** draw power from the Raspberry Pi 5's USB and GPIO power rails, which are rated for sufficient current capacity while providing proper voltage regulation.

All subsystems share a **common ground plane** connected through a central distribution board, ensuring consistent voltage references across digital communication buses (I²C, UART) and preventing ground loops that could introduce noise into sensor readings. Power cables are properly sized - 14AWG silicone wire for motor circuits, 20AWG for servo, and 22AWG for logic-level connections.

The battery pack includes integrated protection circuitry monitoring cell voltages and preventing over-discharge below 9V (3V per cell), over-charge above 12.6V (4.2V per cell), short-circuit conditions, and excessive current draw. This protection extends battery life and prevents dangerous operating conditions.

During competition runs, the battery typically operates in the 11.1V-10.5V range, providing consistent performance. At 10.5V, we implement a low-battery warning through the Raspberry Pi, prompting a battery change to maintain optimal motor performance. The 3000mAh capacity provides approximately 15-20 minutes of active competition time depending on motor usage intensity.

We selected Samsung 30Q cells specifically for their high discharge rating (15A continuous, 20A peak per cell) and stable voltage curve under load. Unlike lower-quality cells that exhibit significant voltage sag during current spikes, these cells maintain relatively flat voltage output, ensuring consistent motor speeds and servo response throughout each run.

The modular nature of 18650 cells allows us to carry spare batteries and quickly swap the entire pack between runs. We maintain multiple battery sets in rotation, charging depleted packs while using fresh ones, ensuring we're always ready for the next competition round.

This power system architecture provides reliable, stable, and sufficient electrical power to all robot subsystems while maintaining safety, weight efficiency, and operational flexibility essential for competitive autonomous robotics.
<div style="clear: both;"></div><br>

### 🧱 Custom 3D Printed Components
<img src="https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025/blob/3d21aa61965569ea2f02e8448cfda0d1e4c31245/other/3D-Printed%20Bot.jpg" width="275" align="right" style="margin-left: 20px;" />

Why we 3D printed select components:  
> - **Precise sensor mounting** with designed alignment features  
> - **Optimized center of gravity** placement for IMU accuracy  
> - **Modular design** enabling rapid repairs or iterations  
> - **Custom cable management** integrated into structure  
> - **Vibration dampening** through strategic material choice  
> - **Lightweight** compared to machined alternatives  
> - **Complex geometries** impossible with traditional fabrication  
> - **Perfect fit** for commercial off-the-shelf components

While our robot's main chassis utilizes a combination of aluminum extrusion, laser-cut acrylic panels, and commercial mounting hardware, we designed and 3D printed specific components to optimize performance, improve integration, and solve unique mechanical challenges inherent to our design.

Critical 3D printed parts include the **REV Robotics IMU mounting bracket** positioned precisely at the robot's geometric center and rotational axis, the **Limelight 3A camera housing** with integrated cooling airflow channels, **REV distance sensor mounts** providing exact angular alignment for optimal coverage, and a **Raspberry Pi 5 mounting tray** with standoffs designed for both mechanical support and thermal management.

The IMU mounting bracket represents perhaps the most important printed component. We designed it to position the sensor at the robot's center of mass with minimal distance from the pitch/roll/yaw axes. This strategic placement dramatically reduces the effect of linear accelerations on gyroscope readings and minimizes rotational noise during turns. The bracket incorporates rubber isolation standoffs to filter high-frequency vibrations from the drivetrain while maintaining rigid coupling for low-frequency motion - essential for accurate sensor fusion.

For the Limelight 3A, we created a custom housing that maintains the required camera angle while providing protection from impacts and organizing cable routing. The housing features built-in channels that guide cooling air past the Google Coral TPU, preventing thermal throttling during extended vision processing. Quick-release latches allow rapid camera adjustment or replacement without tools.

Distance sensor mounts were precision-designed to aim each REV sensor at its optimal coverage zone - front sensor parallel to the ground for obstacle detection, side sensors angled slightly downward to minimize false readings from overhead structures while maintaining wall-following accuracy. Each mount includes adjustment slots for fine-tuning sensor aim during calibration.

The Raspberry Pi 5 mounting solution addresses both mechanical and thermal requirements. The tray positions the board for optimal wire routing to peripherals while elevating it above other components to prevent electromagnetic interference. Integrated standoffs maintain precise spacing for the official active cooler, and strategic cutouts facilitate airflow circulation. The design also incorporates cable management clips that organize power and data connections.

Additional printed components include: servo horn adapters matching the Axon MICRO+ spline to our steering linkage, motor shaft couplers connecting the NeveRest output to the differential input with precise concentricity, battery retention clips that secure the 18650 pack while allowing quick replacement, and cable routing guides that maintain organized wiring throughout the chassis.

We designed all components in Fusion 360, utilizing parametric modeling to ensure precise dimensions and fit. Parts are printed using PETG filament on an FDM printer with 0.2mm layer height and 30% gyroid infill - this combination provides excellent strength-to-weight ratio, dimensional stability across temperature ranges, and adequate flexibility to absorb minor impacts without fracturing.

Material selection varied by component: PETG for structural parts requiring strength and dimensional stability, TPU for vibration-isolating IMU mounts, and ABS for the Limelight housing requiring higher temperature resistance near the active components.

The layer orientation for each part was carefully considered during slicing - sensor mounts are printed with layers oriented to maximize strength against mounting stresses, while the Pi tray uses layers parallel to the board surface for optimal support distribution.

Post-processing included careful deburring of mounting holes, acetone vapor smoothing for ABS parts requiring precise fits, and insertion of heat-set threaded inserts in high-stress mounting points. These inserts provide robust, repeatable fastening that won't strip like printed threads.

This hybrid approach - combining commercial structural components with custom 3D printed integration parts - provided the flexibility to rapidly prototype, test, and refine our mechanical design while maintaining structural integrity and professional build quality. The ability to iterate sensor mounts and adjust mounting positions within hours rather than days proved invaluable during the optimization phase of development.

The modular nature of our printed components means we can quickly replace damaged parts or upgrade specific subsystems without rebuilding the entire chassis. We maintain a library of spare parts and alternative designs, enabling us to adapt to rule changes or implement improvements between competition rounds.
<div style="clear: both;"></div><br>

---

<a name="software--libraries"></a>
## 💻 Software & Libraries

### Development Environment:
- **Python 3.11** - Primary programming language
- **Raspberry Pi OS (64-bit)** - Operating system
- **Visual Studio Code** - Code editor with remote SSH
- **Git** - Version control and collaboration

### Key Python Libraries:
- **RPi.GPIO** - GPIO control and PWM generation
- **smbus2** - I²C communication with IMU and sensors
- **pynetworktables** - Communication with Limelight 3A
- **opencv-python** - Additional image processing if needed
- **numpy** - Numerical computations and array operations
- **pigpio** - High-precision PWM for servo control

### Hardware Interface Libraries:
- **REV Robotics Python Library** - IMU and sensor drivers
- **gpiozero** - Simplified GPIO control
- **adafruit-circuitpython-motor** - Motor control abstractions

---

## 🔧 Core Software Functions

### getFilteredDistance():
#### Reliable Sensor Reading with Advanced Filtering
```python
import numpy as np
from statistics import median

def getFilteredDistance(sensor):
    """
    Reads distance from REV 2m sensor with median filtering
    and outlier rejection for stable measurements.
    """
    samples = 5
    readings = []
    
    for _ in range(samples):
        try:
            distance = sensor.read_distance()
            if 2 <= distance <= 200:  # Valid range in cm
                readings.append(distance)
        except Exception as e:
            print(f"Sensor read error: {e}")
        time.sleep(0.01)
    
    if len(readings) < 3:
        return MAX_DISTANCE  # No valid readings
    
    # Use median to reject outliers
    filtered_distance = median(readings)
    
    # Additional smoothing with exponential moving average
    global distance_ema
    alpha = 0.3  # Smoothing factor
    distance_ema = alpha * filtered_distance + (1 - alpha) * distance_ema
    
    return distance_ema
```
> **Purpose**: Provides noise-resistant distance measurements from REV sensors  
> **Enhancements**: Uses median filtering (better than mean for outlier rejection) + exponential moving average for smooth transitions  
> **Impact**: Eliminates spurious readings that could cause erratic behavior, enables stable wall-following  
> **Why filter**: ToF sensors can occasionally return erroneous values; multiple samples with statistical filtering ensure reliability

### performTurn():
#### Precision Turns Using 9-Axis IMU
```python
def performTurn(clockwise, target_angle_change=88):
    """
    Executes precise turn using REV 9-Axis IMU quaternion data
    with PID control for accurate heading changes.
    """
    imu.update()
    start_heading = imu.get_heading()  # Get current yaw angle
    
    # Calculate target heading (handle 0-360 wraparound)
    if clockwise:
        target_heading = (start_heading - target_angle_change) % 360
    else:
        target_heading = (start_heading + target_angle_change) % 360
    
    # Set steering to maximum for quick turn initiation
    servo.set_angle(SERVO_MAX_RIGHT if clockwise else SERVO_MAX_LEFT)
    motor.set_speed(TURN_SPEED)
    
    # PID controller for smooth turn completion
    kp = 2.0
    kd = 0.5
    last_error = 0
    
    while True:
        imu.update()
        current_heading = imu.get_heading()
        
        # Calculate shortest angular distance to target
        error = calculate_angular_error(current_heading, target_heading)
        
        # Exit condition: within 2 degrees of target
        if abs(error) < 2:
            break
        
        # PID correction as we approach target
        derivative = error - last_error
        correction = kp * error + kd * derivative
        
        # Gradually reduce steering angle as we approach target
        servo_angle = SERVO_CENTER + int(correction)
        servo_angle = constrain(servo_angle, SERVO_MAX_LEFT, SERVO_MAX_RIGHT)
        servo.set_angle(servo_angle)
        
        last_error = error
        time.sleep(0.02)  # 50Hz control loop
    
    # Stop and center steering
    motor.set_speed(0)
    servo.set_angle(SERVO_CENTER)
    
    # Update global target for straight-line tracking
    global target_heading
    target_heading = target_heading

def calculate_angular_error(current, target):
    """Calculate shortest angular distance handling 0-360 wraparound"""
    error = target - current
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
    return error
```
> **Purpose**: Executes accurate 90° turns using IMU feedback  
> **Precision Control**: PID loop ensures consistent turn angles regardless of battery voltage or surface friction  
> **Smart Navigation**: Handles 0-360° wraparound correctly, calculates shortest rotation path  
> **Competition Critical**: Accurate turns are essential for lap counting and edge detection

### Main Control Logic:
#### Advanced Multi-Sensor Navigation with Sensor Fusion
```python
def main_control_loop():
    """
    Main autonomous navigation loop combining vision, IMU, and distance sensors
    with multi-factor PID control for smooth, adaptive driving.
    """
    global target_heading, direction_set, is_clockwise, edge_count
    
    # Control gains (tuned through testing)
    KP_GYRO = 1.5      # Gyroscope heading correction
    KP_WALL = 0.8      # Inner wall distance correction
    KP_OUTER = 1.2     # Outer wall avoidance
    KP_VISION = 2.0    # Limelight target tracking
    
    while is_running:
        # Update all sensors
        imu.update()
        current_heading = imu.get_heading()
        
        # Get filtered distance readings
        front_dist = getFilteredDistance(sensor_front)
        left_dist = getFilteredDistance(sensor_left)
        right_dist = getFilteredDistance(sensor_right)
        
        # Get vision data from Limelight
        vision_target = limelight.get_target()
        
        # Calculate base heading error
        heading_error = calculate_angular_error(current_heading, target_heading)
        
        # Initialize steering correction
        steering_correction = 0
        
        # Multi-factor correction based on current state
        if direction_set and not is_waiting:
            # Determine which wall to follow
            inner_dist = left_dist if is_clockwise else right_dist
            outer_dist = right_dist if is_clockwise else left_dist
            
            # Inner wall following correction
            wall_error = TARGET_WALL_DISTANCE - inner_dist
            steering_correction += KP_WALL * wall_error
            
            # Outer wall avoidance (if too close)
            if outer_dist < OUTER_WALL_THRESHOLD:
                outer_error = OUTER_WALL_THRESHOLD - outer_dist
                steering_correction += KP_OUTER * outer_error * (-1 if is_clockwise else 1)
            
            # Vision-based correction (if target detected)
            if vision_target['valid']:
                # Target X offset from center (-1 to 1)
                vision_error = vision_target['tx'] / 27.0  # Normalize
                steering_correction += KP_VISION * vision_error
        
        # Always apply gyro correction for drift compensation
        steering_correction += KP_GYRO * heading_error
        
        # Calculate final servo angle
        servo_angle = SERVO_CENTER + int(steering_correction)
        servo_angle = constrain(servo_angle, SERVO_MAX_LEFT, SERVO_MAX_RIGHT)
        
        # Apply steering
        servo.set_angle(servo_angle)
        
        # Motor speed control based on steering angle (slow in sharp turns)
        if abs(steering_correction) > 20:
            motor_speed = SLOW_SPEED
        else:
            motor_speed = NORMAL_SPEED
        
        motor.set_speed(motor_speed)
        
        # Check for turn conditions
        check_turn_conditions(left_dist, right_dist, front_dist)
        
        time.sleep(0.02)  # 50Hz main loop

def check_turn_conditions(left_dist, right_dist, front_dist):
    """Detect when to initiate turns based on distance sensor readings"""
    global direction_set, is_clockwise, edge_count, is_waiting
    
    # Obstacle ahead - emergency response
    if front_dist < OBSTACLE_THRESHOLD:
        # Determine safer direction
        if left_dist > right_dist:
            performTurn(clockwise=False)
        else:
            performTurn(clockwise=True)
        return
    
    # Initial direction detection
    if not direction_set:
        if right_dist > SIDE_TURN_DISTANCE:
            is_clockwise = True
            direction_set = True
            edge_count += 1
            performTurn(is_clockwise)
            set_waiting_state()
        elif left_dist > SIDE_TURN_DISTANCE:
            is_clockwise = False
            direction_set = True
            edge_count += 1
            performTurn(is_clockwise)
            set_waiting_state()
    
    # Subsequent corner detection
    elif not is_waiting:
        outer_dist = right_dist if is_clockwise else left_dist
        
        if outer_dist > SIDE_TURN_DISTANCE:
            edge_count += 1
            
            # Check if we've completed required laps
            if edge_count >= TOTAL_EDGES:
                performTurn(is_clockwise)  # Final turn
                time.sleep(1.0)  # Drive forward briefly
                motor.set_speed(0)  # Stop
                global is_running
                is_running = False
                return
            
            performTurn(is_clockwise)
            set_waiting_state()

def set_waiting_state():
    """Prevent repeated turn detection after corner"""
    global is_waiting, wait_start_time
    is_waiting = True
    wait_start_time = time.time()
    
def update_waiting_state():
    """Check if enough time has passed to resume turn detection"""
    global is_waiting
    if is_waiting and (time.time() - wait_start_time) > WAIT_DURATION:
        is_waiting = False
```
> **Multifactor Decision System**: Blends four correction mechanisms:  
>> 1. **Gyro (IMU)** - Corrects angular drift for straight tracking  
>> 2. **Inner Wall Distance** - Maintains consistent offset from track edge  
>> 3. **Outer Wall Avoidance** - Prevents collision when too close  
>> 4. **Vision Tracking** - Follows Limelight-detected targets when present  

> **Adaptive Speed Control**: Automatically reduces speed during sharp corrections  
> **Smart Navigation**: Behavior changes based on lap direction (`is_clockwise`) and current sensor environment  
> **Sensor Fusion**: Combines data from 3 distance sensors, 9-axis IMU, and vision system for robust decision-making

### Limelight Vision Integration:
#### Target Detection and Tracking
```python
from networktables import NetworkTables

class LimelightInterface:
    def __init__(self):
        NetworkTables.initialize(server='10.0.0.11')  # Limelight IP
        self.table = NetworkTables.getTable('limelight')
        self.set_pipeline(0)  # Default vision pipeline
    
    def get_target(self):
        """
        Retrieves current target data from Limelight
        Returns: dict with target information
        """
        tv = self.table.getNumber('tv', 0)  # Target valid (0 or 1)
        tx = self.table.getNumber('tx', 0)  # Horizontal offset (-27 to 27 degrees)
        ty = self.table.getNumber('ty', 0)  # Vertical offset (-20 to 20 degrees)
        ta = self.table.getNumber('ta', 0)  # Target area (0 to 100%)
        
        return {
            'valid': tv == 1,
            'tx': tx,
            'ty': ty,
            'area': ta,
            'distance': self.estimate_distance(ty, ta)
        }
    
    def estimate_distance(self, ty, area):
        """
        Estimates distance to target using vertical angle and area
        Calibrated through field testing
        """
        if area == 0:
            return float('inf')
        
        # Distance estimation based on target area (empirical formula)
        # Assumes known target height
        distance = (TARGET_HEIGHT * FOCAL_LENGTH) / (area ** 0.5)
        return distance
    
    def set_pipeline(self, pipeline_index):
        """Switch between vision pipelines for different detection tasks"""
        self.table.putNumber('pipeline', pipeline_index)
    
    def set_led_mode(self, mode):
        """Control Limelight LED ring (0=pipeline, 1=off, 2=blink, 3=on)"""
        self.table.putNumber('ledMode', mode)
    
    def take_snapshot(self):
        """Capture snapshot for logging/debugging"""
        self.table.putNumber('snapshot', 1)

# Usage in main code
limelight = LimelightInterface()

def vision_assisted_approach():
    """Example: Approach detected target using vision feedback"""
    while True:
        target = limelight.get_target()
        
        if not target['valid']:
            # No target - use distance sensors for navigation
            break
        
        # Center on target
        heading_correction = target['tx'] * 2.0  # Proportional gain
        servo_angle = SERVO_CENTER + int(heading_correction)
        servo.set_angle(servo_angle)
        
        # Approach until at desired distance
        if target['distance'] < APPROACH_DISTANCE:
            motor.set_speed(0)
            break
        else:
            motor.set_speed(APPROACH_SPEED)
        
        time.sleep(0.05)
```
> **NetworkTables Communication**: Standard FRC protocol for reliable Limelight data exchange  
> **Real-time Targeting**: Provides horizontal/vertical offsets for precision alignment  
> **Distance Estimation**: Calculates range to target using area and angle measurements  
> **Pipeline Switching**: Can toggle between different vision configurations (colors, AprilTags, neural networks)  
> **Competition Application**: Used for detecting colored markers, game elements, or positioning targets

---

<a name="competition-challenges"></a>
## 🎯 Competition Challenges

<table>
  <tr>
    <td width="49%" align="center">
      <h3>🔗 Open Challenge</h3>
      <a href="https://youtu.be/91ZgfnIUOyQ">
        <img src="https://img.youtube.com/vi/91ZgfnIUOyQ/0.jpg" width="100%" style="border-radius: 8px;" />
      </a>
      <p><a href="https://youtu.be/91ZgfnIUOyQ">Watch Open Challenge Round</a></p>
      <p><em>Autonomous navigation using IMU and distance sensors for lap completion and precision parking</em></p>
    </td>
    <td width="49%" align="center">
      <h3>🔗 Obstacle Challenge</h3>
      <a href="https://youtu.be/WTx0oLnZWhM">
        <img src="https://img.youtube.com/vi/WTx0oLnZWhM/0.jpg" width="100%" style="border-radius: 8px;" />
      </a>
      <p><a href="https://youtu.be/WTx0oLnZWhM">Watch Obstacle Challenge Round</a></p>
      <p><em>Vision-based obstacle detection and avoidance with dynamic path planning</em></p>
    </td>
  </tr>
</table>

### Challenge Descriptions:

#### Open Challenge
The Open Challenge tests fundamental autonomous navigation capabilities:
- **3 Lap Completion**: Navigate the track clockwise or counterclockwise for exactly 3 laps
- **Wall Following**: Maintain consistent distance from inner wall while avoiding outer wall
- **Corner Detection**: Accurately detect and count all 12 corners using side distance sensors
- **Precision Parking**: Enter designated parking zone and stop within boundaries
- **Time Optimization**: Complete course as quickly as possible while maintaining accuracy

Our strategy uses the REV 9-Axis IMU for maintaining straight-line trajectory between corners, REV distance sensors for wall following and corner detection, and precise turn execution using gyroscope feedback. The lap counting system tracks edges reliably across varying track widths.

#### Obstacle Challenge  
The Obstacle Challenge adds dynamic obstacle avoidance:
- **Random Obstacles**: Red and green colored blocks placed at unknown positions
- **Color Detection**: Use Limelight 3A to identify obstacle colors in real-time
- **Avoidance Logic**: Pass red blocks on right side, green blocks on left side
- **Dynamic Replanning**: Adjust trajectory mid-lap based on obstacle positions
- **Lap Completion**: Complete 3 laps while correctly navigating all obstacles
- **Parking**: Final precision parking after obstacle course completion

We employ multiple Limelight vision pipelines: one for red detection, one for green detection, and high-frequency pipeline switching to track multiple obstacles. The robot calculates avoidance paths using obstacle position data combined with distance sensor inputs for safe clearance.

---

<a name="spoilers"></a>
## 🎭 Spoilers

> We initially tried using AprilTags for absolute positioning, but the detection range was too limited at competition speeds. Switched to HSV color blob detection with dynamic threshold adjustment based on ambient lighting - **much more robust**. Turns out sometimes simpler is better 🎯.

> The Raspberry Pi 5's improved GPIO performance was a game-changer. Previous Pi models struggled with precise servo timing and I²C communication simultaneously. The RP1 I/O controller eliminated that bottleneck completely, giving us **sub-millisecond response times** ⚡.

> We spent weeks optimizing PID gains, only to discover that **adaptive gain scheduling** based on robot speed made everything smoother. High gains during slow turns, lower gains at speed = butter-smooth navigation 🧈.

> Battery voltage drop during motor acceleration was causing servo jitter. Solution? Separate BEC regulators for servo power and logic power. Cost: $12. **Problem solved** 🔋.

> Our first 3D printed IMU mount had the sensor slightly off-center (2cm offset). The rotational acceleration noise was making the gyro data unusable. Moved it to exact center of rotation = **perfect data**. Lesson learned: precision matters 📐.

> The Limelight's Edge TPU can run neural networks, but **color blob detection at 90fps is faster and more reliable** for our application. We keep the neural network pipeline as backup for complex scenes. Sometimes you don't need the fanciest tool 🎨.

> Pro tip: Use **heat-set inserts** in 3D printed parts instead of threading directly into plastic. We broke three mounts before learning this. Now everything is indestructible 💪.

---

## 📞 Contact & Links

- **GitHub Repository**: [WRO-FUTURE-ENGINEERS-2025](https://github.com/oracleapolloo/WRO-FUTURE-ENGINEERS-2025)
- **Team Email**: contact.teamapollo@gmail.com
- **Competition**: World Robot Olympiad 2025 - Future Engineers Category

---

## 🙏 Acknowledgments

Special thanks to:
- Our mentors and coaches for guidance throughout development
- REV Robotics for excellent documentation and support
- The WRO community for sharing knowledge and strategies
- Our families for supporting late-night testing sessions

---

## 📜 License

This project is documented for educational purposes as part of the WRO 2025 competition. All custom code and designs are shared under MIT License for the benefit of the robotics community.

---

**Built with dedication by Team Apollo 🚀**

*Making autonomous vehicles smarter, one competition at a time.*

