#include "command.h"

double kPFL, kIFL, kDFL,kPFR, kIFR, kDFR, kPBL, kIBL, kDBL, kPBR, kIBR, kDBR;
double frontLeftExpectedSpeed, frontRightExpectedSpeed, backLeftExpectedSpeed, backRightExpectedSpeed;
double frontLeftCommandSpeed, frontRightCommandSpeed, backLeftCommandSpeed, backRightCommandSpeed;
double linear, angular;
PID frontLeftPid(&frontLeftActualSpeed, &frontLeftCommandSpeed,  &frontLeftExpectedSpeed, kPFL, kIFL, kDFL, DIRECT);
PID frontRightPid(&frontRightActualSpeed, &frontRightCommandSpeed, &frontRightExpectedSpeed, kPFR, kIFR, kDFR, DIRECT);
PID backLeftPid(&backLeftActualSpeed, &backLeftCommandSpeed, &backLeftExpectedSpeed, kPBL, kIBL, kDBL, DIRECT); 
PID backRightPid(&backRightActualSpeed, &backRightCommandSpeed, &backRightExpectedSpeed, kPBR, kIBR, kDBR, DIRECT);  
ros::NodeHandle nh;
void receiveCommand(const haruto_msgs::Command& command);

ros::Subscriber<haruto_msgs::Command> twistCommand("command", &receiveCommand);

Command::Command()
{ 
  frontLeftPid.SetMode(AUTOMATIC);
  frontRightPid.SetMode(AUTOMATIC);
  backLeftPid.SetMode(AUTOMATIC);
  backRightPid.SetMode(AUTOMATIC);
  
  nh.initNode();  
  nh.subscribe(twistCommand);
}

void Command::computeSpeedToPWM()
{
  frontLeftPid.SetTunings(kPFL, kIFL, kDFL);
  frontRightPid.SetTunings(kPFR, kIFR, kDFR);
  backLeftPid.SetTunings(kPBL, kIBL, kDBL);
  backRightPid.SetTunings(kPBR, kIBR, kDBR);
  frontLeftPid.Compute();
  frontRightPid.Compute();
  backLeftPid.Compute();
  backRightPid.Compute();

  nh.spinOnce();
  delay(1);
}

void receiveCommand(const haruto_msgs::Command& command)
{
  linear = command.x;
  angular = command.z;

  frontLeftExpectedSpeed = ((2 * linear) - (angular * WHEEL_GAP)) / 2;
  backLeftExpectedSpeed = frontLeftExpectedSpeed;
  frontRightExpectedSpeed = ((2 * linear) + (angular * WHEEL_GAP)) / 2;
  backRightExpectedSpeed = frontRightExpectedSpeed;
}
