#include <Encoder.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

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

#define ENCAF 2 
#define ENCAR A1  
#define ENCBF 3 
#define ENCBR 49 
#define ENCCF 18 
#define ENCCR 31 
#define ENCDF 19 
#define ENCDR 38
#define FRONT_LEFT_ENCODER_CYCLE 610
#define FRONT_RIGHT_ENCODER_CYCLE 1004
#define BACK_LEFT_ENCODER_CYCLE 1320 
#define BACK_RIGHT_ENCODER_CYCLE 691
#define CIRCUMFERENCE_FACTOR 0.18857
#define WHEEL_RADIUS 0.03
#define WHEEL_GAP 0.19

Encoder front_left_encoder(ENCAF, ENCAR), front_right_encoder(ENCBF, ENCBR), back_left_encoder(ENCCF, ENCCR), back_right_encoder(ENCDF, ENCDR);

long last_updated_time = 0;
long front_left_tick = 0, front_right_tick = 0, back_left_tick = 0, back_right_tick = 0;
double front_left_actual_speed = 0, front_right_actual_speed = 0, back_left_actual_speed = 0, back_right_actual_speed = 0;
double front_left_expected_speed = 0, front_right_expected_speed = 0, back_left_expected_speed = 0, back_right_expected_speed = 0;
double front_left_command_speed = 0, front_right_command_speed = 0, back_left_command_speed = 0, back_right_command_speed = 0;

//ros::NodeHandle nh;

void receive_command(const geometry_msgs::Twist& cmd_vel)
{
  double x = cmd_vel.linear.x;
  double z = cmd_vel.angular.z;
  front_left_command_speed = ((2*x) - (z*WHEEL_GAP)) / 2;
  back_left_command_speed = front_left_command_speed;
  front_right_command_speed = ((2*x) + (z*WHEEL_GAP)) / 2;
  back_right_command_speed = front_right_command_speed;
}

void compute_speed()
{
  if(millis() - last_updated_time >= 1000)
  {
    front_left_actual_speed = - ((front_left_encoder.read() - front_left_tick) * CIRCUMFERENCE_FACTOR) / FRONT_LEFT_ENCODER_CYCLE;
    front_right_actual_speed = ((front_right_encoder.read() - front_right_tick) * CIRCUMFERENCE_FACTOR) / FRONT_RIGHT_ENCODER_CYCLE;
    back_left_actual_speed = - ((back_left_encoder.read() - back_left_tick) * CIRCUMFERENCE_FACTOR) / BACK_LEFT_ENCODER_CYCLE;
    back_right_actual_speed = ((back_right_encoder.read() - back_right_tick) * CIRCUMFERENCE_FACTOR) / BACK_RIGHT_ENCODER_CYCLE;

    front_left_tick = front_left_encoder.read();
    front_right_tick = front_right_encoder.read();
    back_left_tick = back_left_encoder.read();
    back_right_tick = back_right_encoder.read();
    last_updated_time = millis();
  }
}

void execute_command()
{
  double x = 0.4, z = 0;
  front_left_command_speed = ((2*x) - (z*WHEEL_GAP)) / 2;
  back_left_command_speed = front_left_command_speed;
  front_right_command_speed = ((2*x) + (z*WHEEL_GAP)) / 2;
  back_right_command_speed = front_right_command_speed;

  long temp = map(front_left_command_speed, 0, 0.5, 0, 255);
  Serial.println(front_left_command_speed);
  Serial.println(temp);
//  Serial.println(map(front_left_command_speed, 0, 0.5, 0, 255));
  
//  analogWrite(PWMA, map(front_left_command_speed, 0, 0.5, 0, 255));
//  analogWrite(PWMB, map(front_right_command_speed, 0, 0.5, 0, 255));
//  analogWrite(PWMC, map(back_left_command_speed, 0, 0.5, 0, 255));
//  analogWrite(PWMD, map(back_right_command_speed, 0, 0.5, 0, 255));
  
  if(front_left_command_speed > 0)
  {
    digitalWrite(MAF, HIGH);
    digitalWrite(MAR, LOW);
    digitalWrite(MCF, HIGH);
    digitalWrite(MCR, LOW);
  }
  else if(front_left_command_speed < 0)
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
  
  if(front_right_command_speed > 0)
  {
    digitalWrite(MBF, HIGH);
    digitalWrite(MBR, LOW);
    digitalWrite(MDF, HIGH);
    digitalWrite(MDR, LOW);
  }
  else if(front_right_command_speed < 0)
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
  
//  nh.initNode();  
//  ros::Subscriber<geometry_msgs::Twist> twist_command("cmd_vel", &receive_command);
//  nh.subscribe(twist_command);
 
  Serial.begin(9600);
}

void loop()
{
  Serial.println("running");
  execute_command();
//  compute_speed();
}
