#ifndef Feedback_h
#define Feedback_h

#include <Encoder.h>
#include "storage.h"
#include <haruto_msgs/Tick.h>

class Feedback
{
  public:
    Feedback();
    void broadcastEncoderTick();
};

#endif
