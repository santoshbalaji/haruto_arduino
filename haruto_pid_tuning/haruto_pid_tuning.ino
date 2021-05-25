#include "wheel.h"
#include "velocity.h"
#include "odom.h"

Velocity velocity;
Wheel wheel;
Odom odom;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  velocity.computeSpeedToPWM();
  wheel.executeCommand();
  odom.computeSpeed();
}
