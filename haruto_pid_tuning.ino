#include "command.h"
#include "wheel.h"
#include "odom.h"
#include "feedback.h"

extern Command command;
extern Wheel wheel;
extern Odom odom;
extern Feedback feedback;

void setup()
{
  Command command;
  Wheel wheel;
  Odom odom;
  Feedback feedback;
  Serial.begin(115200);
}

void loop()
{
  command.computeSpeedToPWM();
  wheel.executeCommand();
  odom.computeSpeed();
  feedback.broadcastVelocity();
}
