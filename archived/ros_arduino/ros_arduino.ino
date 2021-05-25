#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;
bool toggle = true;

void messageCb(const std_msgs::Empty& toggle_msg) 
{
  Serial.println("received");
  if(toggle)
  {
    toggle = false;
    digitalWrite(13, HIGH);
  }
  else
  {
    toggle = true;
    digitalWrite(13, LOW);
  }
}

ros::Subscriber<std_msgs::Empty> sub("operation_wilmer", &messageCb);

void setup()
{
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(115200);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}
