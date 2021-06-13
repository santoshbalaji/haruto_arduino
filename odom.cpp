#include "odom.h"

Encoder frontLeftEncoder(ENCAF, ENCAR), frontRightEncoder(ENCBF, ENCBR), backLeftEncoder(ENCCF, ENCCR), backRightEncoder(ENCDF, ENCDR);
double frontLeftActualSpeed, frontRightActualSpeed, backLeftActualSpeed, backRightActualSpeed;
double x, y, theta;
long frontLeftTick, frontRightTick, backLeftTick, backRightTick, tickUpdatedTime = millis();
haruto_msgs::Position position;
ros::Publisher positionPublisher("feedback_position", &position);

Odom::Odom()
{
  nh.advertise(positionPublisher);
}

void Odom::computeSpeed()
{
  if(millis() - tickUpdatedTime >= 1000)
  {
    if(frontLeftExpectedSpeed > 0)
    {
      frontLeftActualSpeed = abs((double)(frontLeftEncoder.read() - frontLeftTick) / FRONT_LEFT_ENCODER_FORWARD_CYCLE);  
      backLeftActualSpeed = abs((double)(backLeftEncoder.read() - backLeftTick) / BACK_LEFT_ENCODER_FORWARD_CYCLE);
    }
    else if(frontLeftExpectedSpeed < 0)
    {
      frontLeftActualSpeed = abs((double)(frontLeftEncoder.read() - frontLeftTick) / FRONT_LEFT_ENCODER_REVERSE_CYCLE) * 1;  
      backLeftActualSpeed = abs((double)(backLeftEncoder.read() - backLeftTick) / BACK_LEFT_ENCODER_REVERSE_CYCLE) * 1;      
    }
    else
    {
      frontLeftActualSpeed = 0;
      backLeftActualSpeed = 0;
    }

    if(frontRightExpectedSpeed > 0)
    {
      frontRightActualSpeed = abs((double)(frontRightEncoder.read() - frontRightTick) / FRONT_RIGHT_ENCODER_FORWARD_CYCLE) * 1;
      backRightActualSpeed = frontRightActualSpeed;
    }
    else if(frontRightExpectedSpeed < 0)
    {
      frontRightActualSpeed = abs((double)(frontRightEncoder.read() - frontRightTick) / FRONT_RIGHT_ENCODER_REVERSE_CYCLE) * 1;
      backRightActualSpeed = frontRightActualSpeed;    
    }
    else
    {
      frontRightActualSpeed = 0;
      backRightActualSpeed = 0;
    }

    frontLeftTick = frontLeftEncoder.read();
    frontRightTick = frontRightEncoder.read();
    backLeftTick = backLeftEncoder.read();
    backRightTick = backRightEncoder.read();
    tickUpdatedTime = millis();
    
    double dL = ((frontLeftExpectedSpeed > 0 ? frontLeftActualSpeed : -frontLeftActualSpeed) + (backLeftExpectedSpeed > 0 ? backLeftActualSpeed : -backLeftActualSpeed)) / 2; 
    double dR = ((frontRightExpectedSpeed > 0 ? frontRightActualSpeed : -frontRightActualSpeed)+ (backRightExpectedSpeed > 0 ? backRightActualSpeed : -backRightActualSpeed)) / 2; 
    double dC = (dL + dR) / 2;
    double degree = (((dL - dR) * WHEEL_RADIUS) / WHEEL_GAP) * (180 / 3.14);
    
    theta = (((dL - dR) * WHEEL_RADIUS) / WHEEL_GAP) + degree;
    x = x + (dC * cos(theta) * WHEEL_RADIUS);
    y = y + (dC * sin(theta) * WHEEL_RADIUS);

    position.x = x;
    position.y = y;
    position.theta = theta;
    positionPublisher.publish(&position);
  }
}
