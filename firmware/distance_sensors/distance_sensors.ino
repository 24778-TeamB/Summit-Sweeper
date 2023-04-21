#include <stdint.h>
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include "ProjectDef.h"
#include "IRsensor.h"

#define NUM_SENSORS 6

ros::NodeHandle nh;

std_msgs::UInt8MultiArray arr_msg;
ros::Publisher dist("ir_sensor", &arr_msg);

uint8_t distance[NUM_SENSORS] = { 0 };

void setup(void)
{ 
  // Configure sensors
  initIR();

  // Initialize ROS
  nh.initNode();
  nh.advertise(dist);
}

void loop(void)
{
	
  get_IR_readings(CENTER, &distance[0], &distance[1]);
  get_IR_readings(REAR, &distance[2], &distance[3]);
  get_IR_readings(SIDE, &distance[4], &distance[5]);

  arr_msg.data = distance;
  arr_msg.data_length = NUM_SENSORS;
  dist.publish(&arr_msg);
  nh.spinOnce();
  delay(100);
}
