#ifndef Task_h
#define Task_h

#include "storage.h"
#include <haruto_msgs/Command.h>

class Task
{
  public:
    Task();
    void executeCommand();
    void activateROSSpin();
};

#endif
