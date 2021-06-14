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
  task.executeCommand();
  feedback.broadcastEncoderTick();
  task.activateROSSpin(); 
}
