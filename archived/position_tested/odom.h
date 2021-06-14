#ifndef Odom_h
#define Odom_h

#include "storage.h"
#include <haruto_msgs/Position.h>
#include <Encoder.h>

class Odom
{
  public:
    Odom();
    void computeSpeed();
};

#endif
