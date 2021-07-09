#ifndef Feedback_h
#define Feedback_h

#include <Encoder.h>
#include "storage.h"
#include <haruto_msgs/Tick.h>
#include <haruto_msgs/IMU.h>
#include <ICM_20948.h>

class Feedback
{
  public:
    Feedback();
    void broadcast_encoder_tick();
    void broadcast_imu();
};

#endif
