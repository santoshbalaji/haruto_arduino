#include "feedback.h"

Encoder front_left_encoder(ENCAF, ENCAR), front_right_encoder(ENCBF, ENCBR), back_left_encoder(ENCCF, ENCCR), back_right_encoder(ENCDF, ENCDR);
haruto_msgs::Tick tick;
haruto_msgs::IMU imu;
ros::Publisher tick_publisher("diff_tick", &tick);
ros::Publisher imu_publisher("diff_imu", &imu);
long elapsed_time_for_tick = millis(), elapsed_time_for_imu = millis();
ICM_20948_I2C icm;

Feedback::Feedback()
{
  nh.advertise(tick_publisher);
  nh.advertise(imu_publisher);
  Wire.begin();
  Wire.setClock(400000);
  icm.begin(Wire, 1);
}

void Feedback::broadcast_encoder_tick()
{
  if(millis() - elapsed_time_for_tick  >= 100)
  {
    tick.front_left_tick = front_left_encoder.read();
    tick.front_right_tick = front_right_encoder.read();
    tick.back_left_tick = back_left_encoder.read();
    tick.back_right_tick = back_right_encoder.read();
    tick_publisher.publish(&tick); 
    elapsed_time_for_tick = millis();
  }
}

void Feedback::broadcast_imu()
{
  if(millis() - elapsed_time_for_imu >= 100)
  {
    if(icm.dataReady())
    {
      icm.getAGMT();
      imu.raw_acc_x = icm.agmt.acc.axes.x;
      imu.raw_acc_y = icm.agmt.acc.axes.y;
      imu.raw_acc_z = icm.agmt.acc.axes.z;
      imu.raw_gyr_x = icm.agmt.gyr.axes.x;
      imu.raw_gyr_y = icm.agmt.gyr.axes.y;
      imu.raw_gyr_z = icm.agmt.gyr.axes.z;
      imu.raw_mag_x = icm.agmt.mag.axes.x;
      imu.raw_mag_y = icm.agmt.mag.axes.y;
      imu.raw_mag_z = icm.agmt.mag.axes.z;
      imu.raw_temp = icm.agmt.tmp.val;
      imu_publisher.publish(&imu);
    }
    elapsed_time_for_imu = millis();
  }
}
