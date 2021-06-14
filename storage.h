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

extern ros::NodeHandle nh;
extern int frontLeftPWM, frontRightPWM, backLeftPWM, backRightPWM, leftState, rightState;

#endif
