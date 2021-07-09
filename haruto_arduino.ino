#include "task.h"
#include "feedback.h"

extern Task task;
extern Feedback feedback; 

void setup()
{
  Task task;
  Feedback feedback;
  Serial.begin(115200);
}

void loop()
{
  task.execute_command();
  feedback.broadcast_encoder_tick();
  feedback.broadcast_imu();
  task.activate_ros_spin(); 
}
