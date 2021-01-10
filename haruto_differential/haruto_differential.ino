#include <ros.h>
#include <std_msgs/String.h>
#include <haruto_msgs/EncoderTick.h>
#include <PID_v1.h>
#include <Encoder.h>

#define PWMA 5
#define MAF A5
#define MAR A4
#define ENCAF 18
#define ENCAR 31 
#define PWMB 9
#define MBF 43
#define MBR 42
#define ENCBF 19
#define ENCBR 38 
#define PWMC 12
#define MCF 35
#define MCR 34
#define ENCCF 3
#define ENCCR 49
#define PWMD 8
#define MDF 37
#define MDR 36
#define ENCDF 2
#define ENCDR 23
#define SPEED 140
#define WHEEL_RADIUS 0.03

Encoder front_left(2, A1), front_right(3, 49), back_left(18, 31), back_right(19, 38);

const double PID_L[] = {0, 0, 0.1};
const double PID_R[] = {0, 0, 0.1};

haruto_msgs::EncoderTick encoder_msg;
ros::NodeHandle nh;
ros::Publisher encoder_info("encoder", &encoder_msg);

void operation(const std_msgs::String& toggle_msg) 
{
  char* a = toggle_msg.data;
  if(a[0] == 'F')
  {
    move_forward();
  }
  else if(a[0] == 'B')
  {
    move_reverse();
  }
  else if(a[0] == 'L')
  {
    turn_left();
  }
  else if(a[0] == 'R')
  {
    turn_right();
  }
  else if(a[0] == 'S')
  {
    brake();
  }  
}

void setup()
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

  analogWrite(PWMA, SPEED);
  analogWrite(PWMB, SPEED);
  analogWrite(PWMC, SPEED);
  analogWrite(PWMD, SPEED);

  nh.initNode();
  nh.advertise(encoder_info);
  ros::Subscriber<std_msgs::String> operation_command("operation", &operation);
  nh.subscribe(operation_command);
  
  Serial.begin(9600);
}

void loop()
{
  notify_encoder_tick();
  nh.spinOnce();
  delay(100);
}

void move_forward()
{
  digitalWrite(MAF, HIGH);
  digitalWrite(MAR, LOW);
  digitalWrite(MBF, HIGH);
  digitalWrite(MBR, LOW);
  digitalWrite(MCF, HIGH);
  digitalWrite(MCR, LOW);
  digitalWrite(MDF, HIGH);
  digitalWrite(MDR, LOW);
}

void move_reverse()
{
  digitalWrite(MAF, LOW);
  digitalWrite(MAR, HIGH);
  digitalWrite(MBF, LOW);
  digitalWrite(MBR, HIGH);
  digitalWrite(MCF, LOW);
  digitalWrite(MCR, HIGH);
  digitalWrite(MDF, LOW);
  digitalWrite(MDR, HIGH);
}

void turn_right()
{
  digitalWrite(MAF, HIGH);
  digitalWrite(MAR, LOW);
  digitalWrite(MBF, LOW);
  digitalWrite(MBR, HIGH);
  digitalWrite(MCF, HIGH);
  digitalWrite(MCR, LOW);
  digitalWrite(MDF, LOW);
  digitalWrite(MDR, HIGH);  
}

void turn_left()
{
  digitalWrite(MAF, LOW);
  digitalWrite(MAR, HIGH);
  digitalWrite(MBF, HIGH);
  digitalWrite(MBR, LOW);
  digitalWrite(MCF, LOW);
  digitalWrite(MCR, HIGH);
  digitalWrite(MDF, HIGH);
  digitalWrite(MDR, LOW);  
}

void brake()
{
  digitalWrite(MAF, HIGH);
  digitalWrite(MAR, HIGH);
  digitalWrite(MBF, HIGH);
  digitalWrite(MBR, HIGH);
  digitalWrite(MCF, HIGH);
  digitalWrite(MCR, HIGH);
  digitalWrite(MDF, HIGH);
  digitalWrite(MDR, HIGH);
}

void compute_speed()
{
  
}

void notify_encoder_tick()
{
  encoder_msg.front_left_encoder = front_left.read();
  encoder_msg.front_right_encoder = front_right.read();
  encoder_msg.back_left_encoder = back_left.read();
  encoder_msg.back_right_encoder = back_right.read();
  encoder_info.publish(&encoder_msg);
}

void notify_speed()
{
  
}
