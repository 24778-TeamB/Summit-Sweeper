#include <stdint.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "ProjectDef.h"
#include "ultrasonic.h"

#define NUM_SENSORS 12

ros::NodeHandle nh;

std_msgs::Float32MultiArray arr_msg;
ros::Publisher dist("ultra_sonic", &arr_msg);

float distance[NUM_SENSORS] = { 0.0 };

void setup(void)
{ 
  // Configure sensors
  init_ultrasonic();

  // Initialize ROS
  nh.initNode();
  nh.advertise(dist);
}

void loop(void)
{
  ultrasonic_t left, right, front, down;
  
  updateHead();
  updateMiddle();
  updateTail();

  left = getLeft();
  right = getRight();
  front = getFront();
  down = getDown();

  distance[0] = left.headModule;
  distance[1] = left.middleModule;
  distance[2] = left.tailModule;
  distance[3] = right.headModule;
  distance[4] = right.middleModule;
  distance[5] = right.tailModule;
  distance[6] = front.headModule;
  distance[7] = front.middleModule;
  distance[8] = front.tailModule;
  distance[9] = down.headModule;
  distance[10] = down.middleModule;
  distance[11] = down.tailModule;

  arr_msg.data = distance;
  arr_msg.data_length = 12;
  dist.publish(&arr_msg);
  nh.spinOnce();
  delay(100);
}
