#include <ros.h>
#include <std_msgs/String.h>

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

Encoder frontLeft(2, A1);
Encoder frontRight(3, 49);
Encoder backLeft(18, 31);
Encoder backRight(19, 38);

long frontLeftPos = 0;
long frontRightPos = 0;
long backLeftPos = 0;
long backRightPos = 0;

long counter_f = 0;
long counter_b = 0;

ros::NodeHandle nh;

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

  ros::Subscriber<std_msgs::String> sub("operation", &operation);
  nh.initNode();
  nh.subscribe(sub);

  Serial.begin(9600);
}

void loop()
{
  nh.spinOnce();
  delay(10);
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
  digitalWrite(MCF, LOW);
  digitalWrite(MCR, HIGH);
  digitalWrite(MDF, HIGH);
  digitalWrite(MDR, LOW);  
}

void turn_left()
{
  digitalWrite(MAF, LOW);
  digitalWrite(MAR, HIGH);
  digitalWrite(MBF, HIGH);
  digitalWrite(MBR, LOW);
  digitalWrite(MCF, HIGH);
  digitalWrite(MCR, LOW);
  digitalWrite(MDF, LOW);
  digitalWrite(MDR, HIGH);  
}

void slide_right()
{
  digitalWrite(MAF, HIGH);
  digitalWrite(MAR, LOW);
  digitalWrite(MBF, LOW);
  digitalWrite(MBR, LOW);
  digitalWrite(MCF, LOW);
  digitalWrite(MCR, LOW);
  digitalWrite(MDF, HIGH);
  digitalWrite(MDR, LOW);  
}

void slide_left()
{
  digitalWrite(MAF, LOW);
  digitalWrite(MAR, LOW);
  digitalWrite(MBF, HIGH);
  digitalWrite(MBR, LOW);
  digitalWrite(MCF, HIGH);
  digitalWrite(MCR, LOW);
  digitalWrite(MDF, LOW);
  digitalWrite(MDR, LOW);  
}

void brake()
{
  digitalWrite(MAF, LOW);
  digitalWrite(MAR, LOW);
  digitalWrite(MBF, LOW);
  digitalWrite(MBR, LOW);
  digitalWrite(MCF, LOW);
  digitalWrite(MCR, LOW);
  digitalWrite(MDF, LOW);
  digitalWrite(MDR, LOW);
}
