
RC-Car ROS2 Control System
โปรเจคนี้เป็นระบบควบคุมรถ RC ที่พัฒนาด้วย ROS2 รองรับทั้งโหมด ควบคุมแบบ Manual (Joystick) และ ควบคุมอัตโนมัติด้วย PID Controller
ระบบถูกออกแบบแบบ Modular เพื่อให้สามารถพัฒนา perception, control และ hardware interface แยกกันได้อย่างชัดเจน
โครงสร้างโปรเจค
rc-car/
│
├── firmware/        # โค้ดไมโครคอนโทรลเลอร์ / ESC / Servo
├── software/        # ROS2 nodes สำหรับ manual และ autonomous control
├── models/          # สมการควบคุม และแบบจำลองทางคณิตศาสตร์
├── images/          # block diagram และ hardware setup
├── docs/            # รายงาน เอกสาร flowchart
└── launch/          # ROS2 launch files
ภาพรวมระบบ (System Overview)
ระบบควบคุมประกอบด้วย
•	Sensor input (เช่น LiDAR)
•	PID Controller สำหรับ steering และ throttle
•	Manual joystick control
•	ESC และ Servo สำหรับควบคุมรถ
ระบบสามารถทำงานได้ 2 โหมด
•	Manual mode : Joystick → รถ
•	Autonomous mode : Target detection → PID → รถ


การควบคุมแบบ Manual
Node joy_to_manual จะรับข้อมูลจาก
/joy
และส่งคำสั่งไปยัง
•	/cmd_esc_manual (ควบคุมความเร็ว)
•	/cmd_s1_manual (ควบคุมเลี้ยว)
การควบคุมแบบ Autonomous
ในโหมดอัตโนมัติ ระบบจะใช้ค่า error จากการติดตาม target
และคำนวณคำสั่ง steering และ throttle ด้วย PID Controller
Hardware ที่ใช้
•	RC Car Platform
•	ESC Motor Controller
•	Steering Servo
•	LiDAR Sensor
•	ROS2 Computer / Embedded Controller
•	Joystick Controller
แผนผังการต่ออุปกรณ์ดูได้ที่
images/hardware_setup.png

Diagram ที่ใช้ในระบบ
•	Block Diagram : images/block_diagram.png
•	PID Control Diagram : images/PID.png
•	Hardware Setup : images/hardware_setup.png
คุณสมบัติเด่น
•	โครงสร้าง ROS2 แบบ Modular
•	PID control สำหรับ steering และ throttle
•	รองรับ manual override
•	Smooth ESC ramp control
•	สามารถต่อยอด perception และ navigation ได้
งานพัฒนาในอนาคต
•	Autonomous navigation เต็มรูปแบบ
•	Obstacle avoidance
•	Sensor fusion (LiDAR + Camera)
•	Adaptive PID tuning



โปรเจกต์นี้จัดทำขึ้นเพื่อการศึกษาและพัฒนาในสาย Robotics และ Control System
