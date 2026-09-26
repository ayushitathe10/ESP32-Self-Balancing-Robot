🤖 Self-Balancing Robot with PID Control

A two-wheeled self-balancing robot built using ESP32, MPU6050, PID control, TB6612FNG motor driver, and DC motors.

The robot uses real-time tilt feedback to continuously adjust the direction and speed of its motors and maintain an upright position.

📌 Project Overview

A two-wheeled balancing robot behaves like an inverted pendulum, making it inherently unstable. Even a small tilt can cause the robot to fall.

To solve this problem, a closed-loop PID control system is used.

The system continuously:

Measures the robot's orientation using the MPU6050.
Obtains the pitch angle using the MPU6050's Digital Motion Processor (DMP).
Compares the measured angle with the desired upright angle.
Uses a PID controller to calculate the required correction.
Sends the correction to the motors through the TB6612FNG motor driver.
Continuously repeats this process to maintain balance.
Control Flow
        MPU6050
           ↓
        DMP
           ↓
      Pitch Angle
           ↓
      PID Controller
           ↓
    Motor Direction
       + PWM
           ↓
      TB6612FNG
           ↓
       DC Motors
           ↓
    Robot Movement
           ↓
        MPU6050
🛠️ Hardware Used
Component	Purpose
ESP32	Main microcontroller and control processing
MPU6050	Measures acceleration and angular velocity
TB6612FNG	Dual H-bridge motor driver
DC Motors	Drive the robot's wheels
Wheels	Provide movement and balance
Battery	Power supply
💻 Software & Technologies
C/C++
Arduino Framework
ESP32
MPU6050
PID Control
I2C Communication
PWM Motor Control
Interrupts
Digital Motion Processor (DMP)
⚙️ How It Works
1. MPU6050

The MPU6050 is a 6-axis IMU containing:

3-axis accelerometer
3-axis gyroscope

The accelerometer measures acceleration and the gyroscope measures angular velocity.

The MPU6050's Digital Motion Processor (DMP) is used to process motion information and obtain orientation data.

The DMP provides orientation information using a quaternion, which is then converted into:

Yaw
Pitch
Roll

For this robot, the pitch angle is used as the main feedback signal because it represents the forward/backward tilt of the robot.

🎛️ PID Control

The robot uses a PID controller to continuously correct its tilt.

The PID equation is:

[
u(t)=K_Pe(t)+K_I\int e(t)dt+K_D\frac{de(t)}{dt}
]

Where:

P (Proportional) → Responds to the current tilt error
I (Integral) → Handles accumulated error
D (Derivative) → Responds to the rate of change of error

The PID output determines both the direction and PWM magnitude of the motor command.

PID Control Flow
Desired Angle
      ↓
    Error
      ↓
     PID
      ↓
Motor Correction
      ↓
Robot Movement
      ↓
New Tilt Angle
      ↓
   Feedback
🚗 Motor Control

The ESP32 controls the motors through the TB6612FNG motor driver.

The ESP32 provides:

Direction signals
PWM signals

The TB6612FNG handles the required motor current and drives the two DC motors.

ESP32
  │
  ├── Direction
  │
  └── PWM
       ↓
  TB6612FNG
       ↓
   DC Motors
Motor Direction

The direction pins determine whether the motors rotate forward or backward.

The PWM signal controls the motor speed.

The PID output determines which direction the robot should move and how strongly it should respond.

🔄 Closed-Loop Feedback

The robot uses closed-loop control rather than running the motors at a fixed speed.

          Desired Angle
                ↓
             Compare
                ↓
        ┌───────┴───────┐
        │               │
        │          Actual Angle
        │               ↑
        ↓               │
       Error          MPU6050
        ↓
       PID
        ↓
   Motor Command
        ↓
      Motors
        ↓
  Robot Movement
        ↓
     MPU6050

This allows the robot to continuously react to changes in its tilt.

🧠 Digital Motion Processor (DMP)

The DMP (Digital Motion Processor) is an internal processing unit of the MPU6050.

It processes the accelerometer and gyroscope data and provides orientation information.

In the code:

mpu.dmpGetQuaternion(&q, fifoBuffer);
mpu.dmpGetGravity(&gravity, &q);
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

The pitch angle is then extracted and used as the PID input:

input = ypr[1] * 180 / M_PI + 180;
🔧 PID Tuning

The PID parameters were tuned experimentally to obtain stable balancing.

The controller needs to balance several factors:

Fast response
Low oscillation
Stable recovery
Minimum steady-state error

The PID parameters depend on the physical characteristics of the robot, including:

Robot weight
Center of mass
Motor torque
Wheel size
Battery voltage
Sensor placement
🔌 Pin Configuration
MPU6050
MPU6050	ESP32
SDA	GPIO 21
SCL	GPIO 22
INT	GPIO 27
VCC	3.3V
GND	GND
TB6612FNG
TB6612FNG	ESP32
AIN1	GPIO 25
AIN2	GPIO 26
PWMA	GPIO 14
BIN1	GPIO 32
BIN2	GPIO 33
PWMB	GPIO 13
STBY	GPIO 12

Pin assignments can be changed according to the actual hardware configuration.

🚀 Getting Started
1. Clone the repository
git clone https://github.com/yourusername/self-balancing-robot.git
2. Open the project

Open the project using Arduino IDE or PlatformIO.

3. Install Required Libraries

Install the required:

I2Cdev library
MPU6050 library
PID library
4. Connect the Hardware

Connect the ESP32, MPU6050, TB6612FNG and DC motors according to the pin configuration.

5. Upload the Code

Select the appropriate ESP32 board and upload the program.

6. Tune the PID

Initially test the robot with the wheels lifted from the ground and tune the PID parameters carefully.

⚠️ Safety Notes
Test the robot with the wheels lifted initially.
Do not connect motors directly to ESP32 GPIO pins.
Use a suitable external power supply for the motors.
Make sure the ESP32 and motor driver have a common ground.
Keep the robot supported during initial PID tuning
