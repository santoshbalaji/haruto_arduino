#ifndef Command_h
#define Command_h

#include "storage.h"
#include <haruto_msgs/Command.h>
#include <PID_v1.h>

class Command
{
  public:
    Command();
    void computeSpeedToPWM();
};

#endif
