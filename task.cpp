#include "task.h"

int frontLeftPWM, frontRightPWM, backLeftPWM, backRightPWM, leftState, rightState;
ros::NodeHandle nh;
void receiveCommand(const haruto_msgs::Command& command);
ros::Subscriber<haruto_msgs::Command> twistCommand("command", &receiveCommand);

Task::Task()
{
  pinMode(PWMA, OUTPUT);
  pinMode(MAF, OUTPUT);
  pinMode(MAR, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(MBF, OUTPUT);
  pinMode(MBR, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(MCF, OUTPUT);
  pinMode(MCR, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(MDF, OUTPUT);
  pinMode(MDR, OUTPUT);

  nh.initNode();  
  nh.subscribe(twistCommand);
}

void Task::executeCommand()
{
  analogWrite(PWMA, frontLeftPWM);
  analogWrite(PWMB, frontRightPWM);
  analogWrite(PWMC, backLeftPWM);
  analogWrite(PWMD, frontRightPWM);
  
  if(leftState == 1)
  {   
    digitalWrite(MAF, HIGH);
    digitalWrite(MAR, LOW);
    digitalWrite(MCF, HIGH);
    digitalWrite(MCR, LOW);
  }
  else if(leftState == -1)
  {
    digitalWrite(MAF, LOW);
    digitalWrite(MAR, HIGH);
    digitalWrite(MCF, LOW);
    digitalWrite(MCR, HIGH);  
  }
  else
  {
    digitalWrite(MAF, HIGH);
    digitalWrite(MAR, HIGH);
    digitalWrite(MCF, HIGH);
    digitalWrite(MCR, HIGH);    
  }
  
  if(rightState == 1)
  {
    digitalWrite(MBF, HIGH);
    digitalWrite(MBR, LOW);
    digitalWrite(MDF, HIGH);
    digitalWrite(MDR, LOW);
  }
  else if(rightState == -1)
  {
    digitalWrite(MBF, LOW);
    digitalWrite(MBR, HIGH);
    digitalWrite(MDF, LOW);
    digitalWrite(MDR, HIGH);  
  }
  else
  {
    digitalWrite(MBF, HIGH);
    digitalWrite(MBR, HIGH);
    digitalWrite(MDF, HIGH);
    digitalWrite(MDR, HIGH);    
  }
}

void Task::activateROSSpin()
{
  nh.spinOnce();
  delay(1);
}

void receiveCommand(const haruto_msgs::Command& command)
{
  frontLeftPWM = command.frontLeftPWM;
  frontRightPWM = command.frontRightPWM;
  backLeftPWM = command.backLeftPWM;
  backRightPWM = command.backRightPWM;
  leftState = command.leftState;
  rightState = command.rightState;
}
