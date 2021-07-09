#ifndef Task_h
#define Task_h

#include "storage.h"
#include <haruto_msgs/PWM.h>

class Task
{
  public:
    Task();
    void execute_command();
    void activate_ros_spin();
};

#endif
