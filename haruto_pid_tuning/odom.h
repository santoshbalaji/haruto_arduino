#ifndef Odom_h
#define Odom_h

#include <Encoder.h>
#include "storage.h"

class Odom
{
  public:
    Odom();
    void computeSpeed();
  private:
    long frontLeftTick = 0;
    long frontRightTick = 0;
    long backLeftTick = 0;
    long backRightTick = 0;
    long lastUpdatedTime = millis();
    void computeOdomRos();
};

#endif
