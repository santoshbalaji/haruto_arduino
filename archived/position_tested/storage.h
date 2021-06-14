#ifndef Storage_h
#define Storage_h

#define ENCAF 2 
#define ENCAR A1  
#define ENCBF 3 
#define ENCBR 49 
#define ENCCF 18 
#define ENCCR 31 
#define ENCDF 19 
#define ENCDR 38

#define FRONT_LEFT_ENCODER_REVERSE_CYCLE 3028
#define FRONT_RIGHT_ENCODER_REVERSE_CYCLE 4500
#define BACK_LEFT_ENCODER_REVERSE_CYCLE 8470
#define BACK_RIGHT_ENCODER_REVERSE_CYCLE 1538

#define FRONT_LEFT_ENCODER_FORWARD_CYCLE 3010
#define FRONT_RIGHT_ENCODER_FORWARD_CYCLE 4990
#define BACK_LEFT_ENCODER_FORWARD_CYCLE 8222 
#define BACK_RIGHT_ENCODER_FORWARD_CYCLE 2776

#define CIRCUMFERENCE_FACTOR 0.18857
#define WHEEL_RADIUS 0.03
#define WHEEL_GAP 0.19

#define PWMA 5
#define MAF A5
#define MAR A4
#define PWMB 9
#define MBF 43
#define MBR 42
#define PWMC 12
#define MCF 35
#define MCR 34
#define PWMD 8
#define MDF 37
#define MDR 36

#include "Arduino.h"
#include <ros.h>

extern double frontLeftActualSpeed, frontRightActualSpeed, backLeftActualSpeed, backRightActualSpeed;
extern double frontLeftExpectedSpeed, frontRightExpectedSpeed, backLeftExpectedSpeed, backRightExpectedSpeed;
extern double frontLeftCommandSpeed, frontRightCommandSpeed, backLeftCommandSpeed, backRightCommandSpeed;
extern double kPFL, kIFL, kDFL,kPFR, kIFR, kDFR, kPBL, kIBL, kDBL, kPBR, kIBR, kDBR;
extern double linear, angular;
extern ros::NodeHandle nh;
extern double x, y, theta;
extern long frontLeftTick, frontRightTick, backLeftTick, backRightTick, tickUpdatedTime;
extern int leftState, rightState;

#endif
