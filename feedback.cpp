#include "feedback.h"

haruto_msgs::Velocity velocity;
haruto_msgs::PID pid;
ros::Publisher velocityPublisher("feedback_velocity", &velocity);
void setParameters(const haruto_msgs::PID& pid);

ros::Subscriber<haruto_msgs::PID> pidParametersCommand("pid_configuration", &setParameters);

Feedback::Feedback()
{ 
  nh.advertise(velocityPublisher);
  nh.subscribe(pidParametersCommand);
}

void Feedback::broadcastVelocity()
{
  velocity.frontLeftExpectedSpeed = frontLeftExpectedSpeed;
  velocity.frontRightExpectedSpeed = frontRightExpectedSpeed;
  velocity.backLeftExpectedSpeed = backLeftExpectedSpeed;
  velocity.backRightExpectedSpeed = backRightExpectedSpeed;
  velocity.frontLeftActualSpeed = frontLeftActualSpeed;
  velocity.frontRightActualSpeed = frontRightActualSpeed;
  velocity.backLeftActualSpeed = backLeftActualSpeed;
  velocity.backRightActualSpeed = backRightActualSpeed;
  velocityPublisher.publish(&velocity);
}

void setParameters(const haruto_msgs::PID& pid)
{
  kPFL = pid.pfl;
  kIFL = pid.ifl;
  kDFL = pid.dfl;
  kPFR = pid.pfr;
  kIFR = pid.ifr;
  kDFR = pid.dfr;
  kPBL = pid.pbl;
  kIBL = pid.ibl;
  kDBL = pid.dbl;
  kPBR = pid.pbr;
  kIBR = pid.ibr;
  kDBR = pid.dbr;
}
