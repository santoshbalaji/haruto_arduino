#ifndef Velocity_h
#define Velocity_h

#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include "storage.h"

class Velocity
{
  public:
    Velocity();
    void computeSpeedToPWM();
};

#endif
