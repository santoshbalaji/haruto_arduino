#include "command.h"

double kPFL, kIFL, kDFL,kPFR, kIFR, kDFR, kPBL, kIBL, kDBL, kPBR, kIBR, kDBR;
double frontLeftExpectedSpeed, frontRightExpectedSpeed, backLeftExpectedSpeed, backRightExpectedSpeed;
double frontLeftCommandSpeed, frontRightCommandSpeed, backLeftCommandSpeed, backRightCommandSpeed;
double linear, angular;
int leftState = 0, rightState = 0;
PID frontLeftPid(&frontLeftActualSpeed, &frontLeftCommandSpeed,  &frontLeftExpectedSpeed, kPFL, kIFL, kDFL, DIRECT);
PID frontRightPid(&frontRightActualSpeed, &frontRightCommandSpeed, &frontRightExpectedSpeed, kPFR, kIFR, kDFR, DIRECT);
PID backLeftPid(&backLeftActualSpeed, &backLeftCommandSpeed, &backLeftExpectedSpeed, kPBL, kIBL, kDBL, DIRECT); 
ros::NodeHandle nh;
void receiveCommand(const haruto_msgs::Command& command);

ros::Subscriber<haruto_msgs::Command> twistCommand("command", &receiveCommand);

Command::Command()
{ 
  frontLeftPid.SetMode(AUTOMATIC);
  frontRightPid.SetMode(AUTOMATIC);
  backLeftPid.SetMode(AUTOMATIC);
  
  nh.initNode();  
  nh.subscribe(twistCommand);
}

void Command::computeSpeedToPWM()
{
  frontLeftPid.SetTunings(kPFL, kIFL, kDFL);
  frontRightPid.SetTunings(kPFR, kIFR, kDFR);
  backLeftPid.SetTunings(kPBL, kIBL, kDBL);
  frontLeftPid.Compute();
  frontRightPid.Compute();
  backLeftPid.Compute();

  nh.spinOnce();
  delay(1);
}

void receiveCommand(const haruto_msgs::Command& command)
{
  linear = command.x;
  angular = command.z;

  frontLeftExpectedSpeed = (linear - ((angular * WHEEL_GAP) / 2)) / WHEEL_RADIUS;
  frontLeftExpectedSpeed = map((frontLeftExpectedSpeed * 100), -3650, 3650, -800, 800);
  frontLeftExpectedSpeed = frontLeftExpectedSpeed / 1000;
  if(frontLeftExpectedSpeed < 0)
  {
    leftState = -1;
    frontLeftExpectedSpeed = abs(frontLeftExpectedSpeed);
  }
  else if(frontLeftExpectedSpeed > 0)
  {
    leftState = 1;
  }
  else
  {
    leftState = 0;
  }
  backLeftExpectedSpeed = frontLeftExpectedSpeed;
  frontRightExpectedSpeed = (linear + ((angular * WHEEL_GAP) / 2)) / WHEEL_RADIUS;
  frontRightExpectedSpeed = map((frontRightExpectedSpeed * 100), -3650, 3650, -800, 800);
  frontRightExpectedSpeed = frontRightExpectedSpeed / 1000;
  if(frontRightExpectedSpeed < 0)
  {
    rightState = -1;
    frontRightExpectedSpeed = abs(frontRightExpectedSpeed);
  }
  else if(frontRightExpectedSpeed > 0)
  {
    rightState = 1;
  }
  else
  {
    rightState = 0;
  }
  backRightExpectedSpeed = frontRightExpectedSpeed;

  char result[8]; 
  dtostrf(frontLeftExpectedSpeed, 6, 2, result);
  nh.loginfo(result);
}
