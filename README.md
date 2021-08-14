# Haruto-arduino

The following arduino project is a base controller for custom made four wheel differential drive robot powered by arduino and ROS 

## Hardware required 
#### 1) Ardunino Mega 2560
<img src="https://github.com/santoshbalaji/haruto-arduino/blob/master/img/2.jpg"  width="400" height="300" />

#### 2) Dual motor driver shield for Mega 2560 compatible with encoder connector
<img src="https://github.com/santoshbalaji/haruto-arduino/blob/master/img/3.webp" width="400" height="300" />

#### 3) Motor with encoders 
<img src="https://github.com/santoshbalaji/haruto-arduino/blob/master/img/4.webp" width="400" height="300" />

## How to connect to ROS from arduino 

#### 1) After ROS installation the following package is required to be installed 
	
http://wiki.ros.org/rosserial_arduino (For connecting ros with serial port)

#### 2) Run the following commands to start the rosserial node (In different terminals)
	
	roscore
	rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=115200
	
	
