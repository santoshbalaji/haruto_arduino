#include "feedback.h"

Encoder front_left_encoder(ENCAF, ENCAR), front_right_encoder(ENCBF, ENCBR), back_left_encoder(ENCCF, ENCCR), back_right_encoder(ENCDF, ENCDR);
haruto_msgs::Tick tick;
haruto_msgs::Reply reply;
ros::Publisher feedback_publisher("diff_feedback", &reply);
long elapsed_time_for_tick = millis();
ICM_20948_I2C icm;

Feedback::Feedback()
{
  nh.advertise(feedback_publisher);
  Wire.begin();
  Wire.setClock(400000);
  icm.begin(Wire, 1);
}

void Feedback::broadcast_feedback()
{
  if(millis() - elapsed_time_for_tick  >= 100)
  {      
    tick.front_left_tick = front_left_encoder.read();
    tick.front_right_tick = front_right_encoder.read();
    tick.back_left_tick = back_left_encoder.read();
    tick.back_right_tick = back_right_encoder.read();
    
    reply.tick = tick;
    feedback_publisher.publish(&reply);
    
    elapsed_time_for_tick = millis();
  }
}
