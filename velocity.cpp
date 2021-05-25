#include "velocity.h"

double kPFL = 5, kIFL = 5, kDFL = 10,kPFR = 5, kIFR = 5, kDFR = 10, kPBL = 5, kIBL = 5, kDBL = 10, kPBR = 5, kIBR = 5, kDBR = 10;
double frontExpectedLeftSpeed, frontExpectedRightSpeed, backExpectedLeftSpeed, backExpectedRightSpeed;
double frontCommandLeftSpeed, frontCommandRightSpeed, backCommandLeftSpeed, backCommandRightSpeed;
double linear, angular;
PID frontLeftPid(&frontActualLeftSpeed, &frontCommandLeftSpeed,  &frontExpectedLeftSpeed, kPFL, kIFL, kDFL, DIRECT);
PID frontRightPid(&frontActualRightSpeed, &frontCommandRightSpeed, &frontExpectedRightSpeed, kPFR, kIFR, kDFR, DIRECT);
PID backLeftPid(&backActualLeftSpeed, &backCommandLeftSpeed, &backExpectedLeftSpeed, kPBL, kIBL, kDBL, DIRECT); 
PID backRightPid(&backActualRightSpeed, &backCommandRightSpeed, &backExpectedRightSpeed, kPBR, kIBR, kDBR, DIRECT);  
ros::NodeHandle nh;

void receive_command(const geometry_msgs::Twist& cmd_vel)
{
  linear = cmd_vel.linear.x;
  angular = cmd_vel.angular.z;

  frontExpectedLeftSpeed = ((2*linear) - (angular*WHEEL_GAP)) / 2;
  backExpectedLeftSpeed = frontExpectedLeftSpeed;
  frontExpectedRightSpeed = ((2*linear) + (angular*WHEEL_GAP)) / 2;
  backExpectedRightSpeed = frontExpectedRightSpeed;
}

ros::Subscriber<geometry_msgs::Twist> twist_command("cmd_vel", &receive_command);

Velocity::Velocity()
{ 
  frontLeftPid.SetMode(AUTOMATIC);
  frontRightPid.SetMode(AUTOMATIC);
  backLeftPid.SetMode(AUTOMATIC);
  backRightPid.SetMode(AUTOMATIC);

  nh.initNode();  
  nh.subscribe(twist_command);
}

void Velocity::computeSpeedToPWM()
{
  frontLeftPid.Compute();
  frontRightPid.Compute();
  backLeftPid.Compute();
  backRightPid.Compute();

  nh.spinOnce();
}
