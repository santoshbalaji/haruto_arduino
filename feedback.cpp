#include "feedback.h"

Encoder frontLeftEncoder(ENCAF, ENCAR), frontRightEncoder(ENCBF, ENCBR), backLeftEncoder(ENCCF, ENCCR), backRightEncoder(ENCDF, ENCDR);
long tickUpdatedTime = millis();
haruto_msgs::Tick tick;
ros::Publisher tickPublisher("feedback_tick", &tick);

Feedback::Feedback()
{
  nh.advertise(tickPublisher);
}

void Feedback::broadcastEncoderTick()
{
  tick.frontLeftTick = frontLeftEncoder.read();
  tick.frontRightTick = frontRightEncoder.read();
  tick.backLeftTick = backLeftEncoder.read();
  tick.backRightTick = backRightEncoder.read();
  tickPublisher.publish(&tick);
}
