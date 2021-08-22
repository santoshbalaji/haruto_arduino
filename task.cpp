#include "task.h"

int front_left_pwm, front_right_pwm, back_left_pwm, back_right_pwm, left_state, right_state;
ros::NodeHandle nh;
void pwm_command(const haruto_msgs::PWM& data);
bool state=false;

ros::Subscriber<haruto_msgs::PWM> pwm_sub("/diff_pwm", &pwm_command);

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
  nh.subscribe(pwm_sub);
}

void Task::execute_command()
{
  if(state)
  {
    analogWrite(PWMA, front_left_pwm);
    analogWrite(PWMB, front_right_pwm);
    analogWrite(PWMC, back_left_pwm);
    analogWrite(PWMD, back_right_pwm);
  
    if(left_state == 1)
    {
      nh.loginfo("left wheels moving in forward direction");         
      digitalWrite(MAF, HIGH);
      digitalWrite(MAR, LOW);
      digitalWrite(MCF, HIGH);
      digitalWrite(MCR, LOW);
    }
    else if(left_state == 2)
    {
      nh.loginfo("left wheels moving in backward direction");
      digitalWrite(MAF, LOW);
      digitalWrite(MAR, HIGH);
      digitalWrite(MCF, LOW);
      digitalWrite(MCR, HIGH);  
    }
    else
    {
      nh.loginfo("left wheels brake applied");
      digitalWrite(MAF, HIGH);
      digitalWrite(MAR, HIGH);
      digitalWrite(MCF, HIGH);
      digitalWrite(MCR, HIGH);    
    }
  
    if(right_state == 1)
    {
      nh.loginfo("right wheels moving in forward direction");      
      digitalWrite(MBF, HIGH);
      digitalWrite(MBR, LOW);
      digitalWrite(MDF, HIGH);
      digitalWrite(MDR, LOW);
    }
    else if(right_state == 2)
    {
      nh.loginfo("right wheels moving in backward direction");
      digitalWrite(MBF, LOW);
      digitalWrite(MBR, HIGH);
      digitalWrite(MDF, LOW);
      digitalWrite(MDR, HIGH);  
    }
    else
    {
      nh.loginfo("right wheels brake applied");
      digitalWrite(MBF, HIGH);
      digitalWrite(MBR, HIGH);
      digitalWrite(MDF, HIGH);
      digitalWrite(MDR, HIGH);    
    }
    state = false;
  }
}

void Task::activate_ros_spin()
{
  nh.spinOnce();
  delay(1);
}

void pwm_command(const haruto_msgs::PWM& data)
{
  front_left_pwm = data.front_left_pwm;
  front_right_pwm = data.front_right_pwm;
  back_left_pwm = data.back_left_pwm;
  back_right_pwm = data.back_right_pwm;
  left_state = data.left_state;
  right_state = data.right_state;
  state = true;
}
