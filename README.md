# haruto-arduino

The following arduino project is a base controller for custom made four wheel differential drive robot powered by arduino and ROS 

## Hardware required 
	1) Arduino Mega2560
	 <img src="https://github.com/santoshbalaji/haruto-arduino/blob/master/img/2.jpg" width="350" />
	2) Dual motor driver shield for Arduino mega 2560 with encoder compatible connectors 
	 <img src="https://github.com/santoshbalaji/haruto-arduino/blob/master/img/3.webp" width="350" />
	3) Motor with encoder 	
	 <img src="https://github.com/santoshbalaji/haruto-arduino/blob/master/img/4.webp" width="350" />

### 1) Storage (header.h)
      This header file is to keep track of commonly used variables across the project 

### 2) Velocity (velocity.h, velocity.cpp)
      This project uses ROSSerial node to get command velocity from navigation stack. Once command velocity is obtained 
      the velocity is converted to individual wheel velocities and then to individual PWM signals. Also PID libraries 
      are used here to self correct errors created in achieving expected velocities. 

### 3) Wheel (wheel.h, wheel.cpp)
      Once individual PWM signals are obtained they are passed to analog pins as PWM signals to maintain speed 
      of the motor.        

### 4) Odom (odom.h, odom.cpp)
      After PWM signals is supplied to motor there are wheel encoders and IMU which takes feedback of distance moved 
      to calculate actual wheel velocities.
