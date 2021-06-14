#ifndef Feedback_h
#define Feedback_h

#include "storage.h"
#include <haruto_msgs/Velocity.h>
#include <haruto_msgs/PID.h>

class Feedback
{
  public:
    Feedback();
    void broadcastVelocity();
};

#endif
