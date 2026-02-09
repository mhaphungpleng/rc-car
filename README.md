# RC-Car ROS2 Control System

โปรเจกต์นี้เป็นระบบควบคุมรถ RC ที่พัฒนาด้วย **ROS2**  
รองรับทั้งโหมดควบคุมแบบ **Manual (Joystick)** และ **Autonomous (PID Controller)**  
ระบบถูกออกแบบแบบ **Modular** เพื่อแยก perception, control และ hardware interface

---

## Project Structure

rc-car/
│
├── firmware/ # โค้ดไมโครคอนโทรลเลอร์ / ESC / Servo
├── software/ # ROS2 nodes สำหรับ manual และ autonomous control
├── models/ # สมการควบคุมและแบบจำลองทางคณิตศาสตร์
├── images/ # block diagram และ hardware setup
├── docs/ # รายงานและเอกสาร
├── launch/ # ROS2 launch files


---

## System Overview

ระบบควบคุมประกอบด้วย

- Sensor input (เช่น LiDAR)
- PID Controller สำหรับ steering และ throttle
- Manual joystick control
- ESC และ Servo สำหรับควบคุมรถ

ระบบสามารถทำงานได้ 2 โหมด

**Manual mode**

**Autonomous mode**

---

## Manual Control

Node `joy_to_manual`

Subscribe:
- `/joy`

Publish:
- `/cmd_esc_manual`
- `/cmd_s1_manual`

---

## Autonomous Control

ระบบใช้ค่า error จาก target  
คำนวณ steering และ throttle ด้วย **PID Controller**

---

## Hardware

- RC Car Platform
- ESC Motor Controller
- Steering Servo
- LiDAR Sensor
- ROS2 Computer / Embedded Controller
- Joystick Controller

Hardware diagram:

---

## Diagrams

- Block Diagram: `images/block_diagram.png`
- PID Control Diagram: `images/PID.png`
- Hardware Setup: `images/hardware_setup.png`

---

## Features

- Modular ROS2 architecture
- PID control สำหรับ steering และ throttle
- รองรับ manual override
- Smooth ESC ramp control
- สามารถต่อยอด perception และ navigation ได้

---

## Future Work

- Autonomous navigation เต็มรูปแบบ
- Obstacle avoidance
- Sensor fusion (LiDAR + Camera)
- Adaptive PID tuning

---

โปรเจกต์นี้จัดทำขึ้นเพื่อการศึกษาและพัฒนาในสาย **Robotics และ Control System**

