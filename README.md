<div align="center">

# 🚗 Autonomous Navigation Car

### Arduino-Based Autonomous Obstacle Avoidance Robot

<p>

![Arduino](https://img.shields.io/badge/Arduino-Uno-00979D?style=for-the-badge&logo=arduino)
![C++](https://img.shields.io/badge/Language-C++-00599C?style=for-the-badge&logo=cplusplus)
![Embedded Systems](https://img.shields.io/badge/Embedded-Systems-success?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Completed-brightgreen?style=for-the-badge)

</p>

An autonomous robotic vehicle capable of detecting and avoiding obstacles in real time using an Arduino Uno, ultrasonic sensor, servo motor, and intelligent obstacle avoidance logic.

</div>

---

# 📖 Overview

The **Autonomous Navigation Car** is a low-cost embedded robotics project designed to navigate independently without human intervention.

The vehicle continuously scans its surroundings using a servo-mounted ultrasonic sensor, detects obstacles, determines the safest direction, and autonomously avoids collisions using real-time decision-making.

The project demonstrates the practical application of embedded systems, robotics, sensor integration, and autonomous navigation.

---

# ✨ Features

- 🚗 Autonomous navigation
- 📡 Real-time obstacle detection
- 🔄 Dynamic obstacle avoidance
- 🎯 Servo-based environmental scanning
- 📏 Ultrasonic distance measurement
- ⚡ Real-time decision making
- 🔙 Automatic reverse maneuver
- ↩ Intelligent left/right path selection
- 🔁 180° turnaround when no path exists
- 🔋 Low-cost hardware implementation

---

# 🏗️ System Architecture

```
              Ultrasonic Sensor
                     │
                     ▼
               Distance Measurement
                     │
                     ▼
              Arduino UNO Controller
                     │
      ┌──────────────┼──────────────┐
      ▼              ▼              ▼
 Servo Motor    Decision Logic   Motor Driver
      │              │              │
      └──────────────┼──────────────┘
                     ▼
               DC Motors
                     │
                     ▼
          Autonomous Navigation
```

---

# ⚙️ Hardware Components

| Component | Purpose |
|-----------|----------|
| Arduino Uno | Main Controller |
| HC-SR04 Ultrasonic Sensor | Obstacle Detection |
| SG90 Servo Motor | Sensor Rotation |
| L298N Motor Driver | Motor Control |
| 4 DC Motors | Vehicle Movement |
| Li-ion Batteries | Power Supply |
| Robot Chassis | Vehicle Platform |

---

# 💻 Software Used

- Arduino IDE
- Embedded C / Arduino C++
- AFMotor Library
- Servo Library
- NewPing Library

---

# 🔄 Working Principle

1. The ultrasonic sensor continuously measures the distance to nearby obstacles.
2. Sensor readings are filtered using median filtering to reduce noise.
3. If an obstacle is detected within the threshold distance, the vehicle stops.
4. The system verifies whether the obstacle is stationary.
5. The servo rotates to scan both left and right directions.
6. The Arduino compares the available distances.
7. The robot selects the clearer path.
8. If no safe path exists, the robot performs a 180° turn.
9. Navigation resumes automatically.

---

# 🔁 Navigation Algorithm

```
Start

↓

Move Forward

↓

Obstacle Detected?

↓

No ─────────► Continue Moving

↓

Yes

↓

Stop Vehicle

↓

Verify Stationary Obstacle

↓

Reverse

↓

Scan Left & Right

↓

Compare Distances

↓

Clear Path?

↓

Turn Left / Turn Right

↓

No

↓

Turn Around (180°)

↓

Continue Navigation
```

---

# 🛠️ Technologies Used

| Technology | Purpose |
|------------|----------|
| Arduino Uno | Microcontroller |
| Embedded C++ | Programming |
| Ultrasonic Sensor | Distance Measurement |
| Servo Motor | Environmental Scanning |
| PWM | Motor Speed Control |
| L298N | Motor Driver |

---

# 📊 Project Highlights

- Autonomous obstacle avoidance
- Servo-controlled environmental scanning
- Median-filtered distance measurements
- PWM-based motor control
- Real-time navigation decisions
- Low-cost robotic implementation
- Modular hardware architecture

---

# 📈 Results

- ✅ Real-time obstacle detection
- ✅ Autonomous path correction
- ✅ Dynamic obstacle avoidance
- ✅ Stable motor control
- ✅ Low-cost implementation
- ✅ Reliable navigation in controlled environments

---

# 📸 Images of the car


<img width="3024" height="4032" alt="IMG-20250327-WA0008" src="https://github.com/user-attachments/assets/b60ee63c-703e-4112-9da4-498ec374d5b4" />
<img width="3024" height="4032" alt="IMG-20250327-WA0007" src="https://github.com/user-attachments/assets/328841d0-0ad5-4285-a53b-ca87beb1ddbf" />
<img width="3024" height="4032" alt="IMG-20250327-WA0005" src="https://github.com/user-attachments/assets/4b3540f9-d87a-4555-a38e-95367ef38a95" />


# 📂 Project Structure

```
Autonomous-Navigation-Car/

│── Arduino_Code/
│── Circuit_Diagram/
│── Images/
│── Project_Report.pdf
│── README.md
│── screenshots/
```

---

# 🚀 How to Run

### Clone Repository

```bash
git clone https://github.com/SammidiNishithReddy/Autonomous-Navigation-Car.git
```

### Open Arduino IDE

Load the Arduino source code.

### Install Libraries

- Servo
- AFMotor
- NewPing

### Upload Code

Select the Arduino Uno board and upload the program.

### Power the Robot

Connect batteries and place the robot on a flat surface.

The robot will begin autonomous navigation automatically.

---

# 💡 Applications

- Autonomous Robots
- Smart Wheelchairs
- Warehouse Automation
- Agricultural Robotics
- Educational Robotics
- Industrial Automation
- Indoor Navigation
- Obstacle Avoidance Systems

---

# 🎯 Skills Demonstrated

- Embedded Systems
- Robotics
- Arduino Programming
- Sensor Integration
- Motor Control
- PWM
- Embedded C++
- Autonomous Navigation
- Real-Time Decision Making
- Hardware Prototyping

---

# 🚧 Limitations

- Limited detection range
- Performance affected by reflective surfaces
- Fixed obstacle avoidance strategy
- Battery runtime limitations
- Not suitable for high-speed navigation

---

# 🔮 Future Enhancements

- LiDAR integration
- Camera-based vision
- Machine Learning navigation
- Reinforcement Learning
- GPS navigation
- Bluetooth/Wi-Fi control
- Mobile application
- ROS integration
- SLAM mapping
- Solar charging

---

# 📊 Project Information

| Category | Details |
|-----------|----------|
| Project Type | Embedded Systems |
| Domain | Robotics |
| Platform | Arduino Uno |
| Programming Language | Embedded C++ |
| Sensors | HC-SR04 Ultrasonic Sensor |
| Motor Driver | L298N |
| Navigation | Autonomous |

---

# ⭐ Support

If you found this project useful:

⭐ Star this repository

🍴 Fork this repository

🐞 Report Issues

---

# 👨‍💻 Authors

### Sammidi Nishith Reddy
### D. Raj Kumar
### A. Abhilash

**B.Tech Computer Science Engineering (Artificial Intelligence)**

Amrita Vishwa Vidyapeetham

### 🌐 Connect with Me

**GitHub**

https://github.com/SammidiNishithReddy

**LinkedIn**

https://www.linkedin.com/in/sammidi-nishith-r%C3%AAddy-0831a5328/

---

# 📜 License

This project is intended for educational and research purposes.

---

<div align="center">

### 🚗 Intelligent Navigation Through Embedded Robotics

⭐ Thank you for visiting this repository!

</div>
