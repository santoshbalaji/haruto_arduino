#include <Encoder.h>
#include <PID_v1.h>

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

#define FRONT_LEFT_ENCODER_FORWARD_CYCLE 370
#define FRONT_RIGHT_ENCODER_FORWARD_CYCLE 580
#define BACK_LEFT_ENCODER_FORWARD_CYCLE 1380 
#define BACK_RIGHT_ENCODER_FORWARD_CYCLE 620
#define FRONT_LEFT_ENCODER_REVERSE_CYCLE 150
#define FRONT_RIGHT_ENCODER_REVERSE_CYCLE 360
#define BACK_LEFT_ENCODER_REVERSE_CYCLE 1320 
#define BACK_RIGHT_ENCODER_REVERSE_CYCLE 710


#define CIRCUMFERENCE_FACTOR 0.18857
#define WHEEL_RADIUS 0.03
#define WHEEL_GAP 0.19

Encoder front_left_encoder(ENCAF, ENCAR), front_right_encoder(ENCBF, ENCBR), back_left_encoder(ENCCF, ENCCR), back_right_encoder(ENCDF, ENCDR);

long last_updated_time = 0;
long front_left_tick = 0, front_right_tick = 0, back_left_tick = 0, back_right_tick = 0;
double front_left_actual_speed = 0, front_right_actual_speed = 0, back_left_actual_speed = 0, back_right_actual_speed = 0;
double front_left_expected_speed = 0, front_right_expected_speed = 0, back_left_expected_speed = 0, back_right_expected_speed = 0;
double front_left_actual_abs_speed = 0, front_right_actual_abs_speed = 0, back_left_actual_abs_speed = 0, back_right_actual_abs_speed = 0;
double front_left_expected_abs_speed = 0, front_right_expected_abs_speed = 0, back_left_expected_abs_speed = 0, back_right_expected_abs_speed = 0;
double front_left_command_speed = 0, front_right_command_speed = 0, back_left_command_speed = 0, back_right_command_speed = 0;

PID front_left_pid(&front_left_actual_abs_speed, &front_left_command_speed,  &front_left_expected_abs_speed, 10, 10, 5, DIRECT);
PID front_right_pid(&front_right_actual_abs_speed, &front_right_command_speed, &front_right_expected_abs_speed, 10, 10, 5, DIRECT);
PID back_left_pid(&back_left_actual_abs_speed, &back_left_command_speed, &back_left_expected_abs_speed, 10, 10, 5, DIRECT); 
PID back_right_pid(&back_right_actual_abs_speed, &back_right_command_speed, &back_right_expected_abs_speed, 10, 10, 5, DIRECT);

double x = 0.2, z = 0;

char buff[50];

void execute_command();
void compute_actual_speed();
char* prepare_printer(double expected_speed, double actual_speed, double command_speed, char* notation);

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

  front_left_pid.SetMode(AUTOMATIC);
  front_right_pid.SetMode(AUTOMATIC);
  back_left_pid.SetMode(AUTOMATIC);
  back_right_pid.SetMode(AUTOMATIC);
  
  Serial.begin(9600);
}

void loop() 
{
  execute_command();
  compute_actual_speed();
}

void execute_command()
{
  front_left_expected_speed = ((2*x) - (z*WHEEL_GAP)) / 2;
  back_left_expected_speed = front_left_expected_speed;
  front_right_expected_speed = ((2*x) + (z*WHEEL_GAP)) / 2;
  back_right_expected_speed = front_right_expected_speed;

  front_left_expected_abs_speed = abs(front_left_expected_speed);
  front_right_expected_abs_speed = abs(front_right_expected_speed);
  back_left_expected_abs_speed = abs(back_left_expected_speed);
  back_right_expected_abs_speed = abs(back_right_expected_speed);

  front_left_pid.Compute();
  front_right_pid.Compute();
  back_left_pid.Compute();
  back_right_pid.Compute();

  analogWrite(PWMA, front_left_command_speed);
  analogWrite(PWMB, front_right_command_speed);
  analogWrite(PWMC, back_left_command_speed);
  analogWrite(PWMD, back_right_command_speed);

  if(front_left_expected_speed > 0)
  {    
    digitalWrite(MAF, HIGH);
    digitalWrite(MAR, LOW);
    digitalWrite(MCF, HIGH);
    digitalWrite(MCR, LOW);
  }
  else if(front_left_expected_speed < 0)
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
  
  if(front_right_expected_speed > 0)
  {
    digitalWrite(MBF, HIGH);
    digitalWrite(MBR, LOW);
    digitalWrite(MDF, HIGH);
    digitalWrite(MDR, LOW);
  }
  else if(front_right_expected_speed < 0)
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

void compute_actual_speed()
{
  if(millis() - last_updated_time >= 1000)
  {
    if(front_left_expected_speed > 0)
    {
        front_left_actual_speed = - ((front_left_encoder.read() - front_left_tick) * CIRCUMFERENCE_FACTOR) / FRONT_LEFT_ENCODER_FORWARD_CYCLE;  
        back_left_actual_speed = - ((back_left_encoder.read() - back_left_tick) * CIRCUMFERENCE_FACTOR) / BACK_LEFT_ENCODER_FORWARD_CYCLE;
    }
    else if(front_left_expected_speed < 0)
    {
        front_left_actual_speed = - ((front_left_encoder.read() - front_left_tick) * CIRCUMFERENCE_FACTOR) / FRONT_LEFT_ENCODER_REVERSE_CYCLE;  
        back_left_actual_speed = - ((back_left_encoder.read() - back_left_tick) * CIRCUMFERENCE_FACTOR) / BACK_LEFT_ENCODER_REVERSE_CYCLE;      
    }

    if(front_right_expected_speed > 0)
    {
      front_right_actual_speed = ((front_right_encoder.read() - front_right_tick) * CIRCUMFERENCE_FACTOR) / FRONT_RIGHT_ENCODER_FORWARD_CYCLE;
      back_right_actual_speed = ((back_right_encoder.read() - back_right_tick) * CIRCUMFERENCE_FACTOR) / BACK_RIGHT_ENCODER_FORWARD_CYCLE;
    }
    else if(front_right_expected_speed < 0)
    {
      front_right_actual_speed = ((front_right_encoder.read() - front_right_tick) * CIRCUMFERENCE_FACTOR) / FRONT_RIGHT_ENCODER_REVERSE_CYCLE;
      back_right_actual_speed = ((back_right_encoder.read() - back_right_tick) * CIRCUMFERENCE_FACTOR) / BACK_RIGHT_ENCODER_REVERSE_CYCLE;      
    }
    
    front_left_actual_abs_speed = abs(front_left_actual_speed);
    front_right_actual_abs_speed = abs(front_right_actual_speed);
    back_left_actual_abs_speed = abs(back_left_actual_speed);
    back_right_actual_abs_speed = abs(back_right_actual_speed);

    front_left_tick = front_left_encoder.read();
    front_right_tick = front_right_encoder.read();
    back_left_tick = back_left_encoder.read();
    back_right_tick = back_right_encoder.read();
    last_updated_time = millis();

    Serial.println("----------- start -------------");
    Serial.println(prepare_printer(front_left_expected_speed, front_left_actual_speed, front_left_command_speed, "FL"));
    Serial.println(prepare_printer(front_right_expected_speed, front_right_actual_speed, front_right_command_speed, "FR"));
    Serial.println(prepare_printer(back_left_expected_speed, back_left_actual_speed, back_left_command_speed, "BL"));
    Serial.println(prepare_printer(back_right_expected_speed, back_right_actual_speed, back_right_command_speed, "BR"));
    Serial.println("----------- stop -------------");
  }
}

char* prepare_printer(double expected_speed, double actual_speed, double command_speed, char* notation)
{
  char expected_speed_str[7], actual_speed_str[7], command_speed_str[7];
  dtostrf((float) expected_speed, 2, 2, expected_speed_str);
  dtostrf((float) actual_speed, 2, 2, actual_speed_str);
  dtostrf((float) command_speed, 2, 2, command_speed_str);
  sprintf(buff, "%s %s %s %s", notation, expected_speed_str, actual_speed_str, command_speed_str);
  return buff;
}
