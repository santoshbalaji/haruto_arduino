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

#define FRONT_RIGHT_ENCODER_REVERSE_CYCLE 740
#define BACK_RIGHT_ENCODER_REVERSE_CYCLE 740
#define FRONT_LEFT_ENCODER_REVERSE_CYCLE 370
#define BACK_LEFT_ENCODER_REVERSE_CYCLE 1320 

#define FRONT_RIGHT_ENCODER_FORWARD_CYCLE 1070
#define BACK_RIGHT_ENCODER_FORWARD_CYCLE 620
#define FRONT_LEFT_ENCODER_FORWARD_CYCLE 450
#define BACK_LEFT_ENCODER_FORWARD_CYCLE 1320 

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
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

extern double frontActualLeftSpeed, frontActualRightSpeed, backActualLeftSpeed, backActualRightSpeed;
extern double frontExpectedLeftSpeed, frontExpectedRightSpeed, backExpectedLeftSpeed, backExpectedRightSpeed;
extern double frontCommandLeftSpeed, frontCommandRightSpeed, backCommandLeftSpeed, backCommandRightSpeed;
extern double kPFL, kIFL, kDFL,kPFR, kIFR, kDFR, kPBL, kIBL, kDBL, kPBR, kIBR, kDBR;
extern double linear, angular;
extern ros::NodeHandle nh;
extern geometry_msgs::TransformStamped t;
extern tf::TransformBroadcaster broadcaster;
extern double x, y, theta;

#endif