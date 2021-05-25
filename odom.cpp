#include "odom.h"

Encoder frontLeftEncoder(ENCAF, ENCAR), frontRightEncoder(ENCBF, ENCBR), backLeftEncoder(ENCCF, ENCCR), backRightEncoder(ENCDF, ENCDR);
double frontActualLeftSpeed, frontActualRightSpeed, backActualLeftSpeed, backActualRightSpeed;
double x, y, theta;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

Odom::Odom()
{
  broadcaster.init(nh);
}

void Odom::computeSpeed()
{
  if(millis() - lastUpdatedTime >= 1000)
  {
    if(frontExpectedLeftSpeed > 0)
    {
        frontActualLeftSpeed = abs(((frontLeftEncoder.read() - frontLeftTick) * CIRCUMFERENCE_FACTOR) / FRONT_LEFT_ENCODER_FORWARD_CYCLE);  
        backActualLeftSpeed = abs(((backLeftEncoder.read() - backLeftTick) * CIRCUMFERENCE_FACTOR) / BACK_LEFT_ENCODER_FORWARD_CYCLE);
    }
    else if(frontExpectedLeftSpeed < 0)
    {
        frontActualLeftSpeed = abs(((frontLeftEncoder.read() - frontLeftTick) * CIRCUMFERENCE_FACTOR) / FRONT_LEFT_ENCODER_REVERSE_CYCLE);  
        backActualLeftSpeed = abs(((backLeftEncoder.read() - backLeftTick) * CIRCUMFERENCE_FACTOR) / BACK_LEFT_ENCODER_REVERSE_CYCLE);      
    }
    else
    {
      frontActualLeftSpeed = 0;
      backActualLeftSpeed = 0;
    }

    if(frontExpectedRightSpeed > 0)
    {
      frontActualRightSpeed = abs(((frontRightEncoder.read() - frontRightTick) * CIRCUMFERENCE_FACTOR) / FRONT_RIGHT_ENCODER_FORWARD_CYCLE);
      backActualRightSpeed = abs(((backRightEncoder.read() - backRightTick) * CIRCUMFERENCE_FACTOR) / BACK_RIGHT_ENCODER_FORWARD_CYCLE);
    }
    else if(frontExpectedRightSpeed < 0)
    {
      frontActualRightSpeed = abs(((frontRightEncoder.read() - frontRightTick) * CIRCUMFERENCE_FACTOR) / FRONT_RIGHT_ENCODER_REVERSE_CYCLE);
      backActualRightSpeed = abs(((backRightEncoder.read() - backRightTick) * CIRCUMFERENCE_FACTOR) / BACK_RIGHT_ENCODER_REVERSE_CYCLE);      
    }
    else
    {
      frontActualRightSpeed = 0;
      backActualRightSpeed = 0;
    }


    
    frontLeftTick = frontLeftEncoder.read();
    frontRightTick = frontRightEncoder.read();
    backLeftTick = backLeftEncoder.read();
    backRightTick = backRightEncoder.read();
    lastUpdatedTime = millis();
    
    double dL = (frontExpectedLeftSpeed > 0 ? frontActualLeftSpeed : -frontActualLeftSpeed + backExpectedLeftSpeed > 0 ? backActualLeftSpeed : -backActualLeftSpeed) / 2; 
    double dR = (frontExpectedRightSpeed > 0 ? frontActualRightSpeed : -frontActualRightSpeed + backExpectedRightSpeed > 0 ? backActualRightSpeed : -backActualRightSpeed) / 2; 
    double dC = (dL + dR) / 2;
    theta = ((dL - dR) / WHEEL_GAP) + theta;
    x = x + (dC * cos(theta));
    y = y + (dC * sin(theta));

    t.header.frame_id = "/odom";
    t.child_frame_id = "/base_link";
  
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = frontActualLeftSpeed;
  
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();
  
    broadcaster.sendTransform(t);
  }
}
