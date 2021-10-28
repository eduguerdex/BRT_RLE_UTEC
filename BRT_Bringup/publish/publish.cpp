#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/MultiArrayLayout.h"
#include <std_msgs/Float32MultiArray.h>

int PI = 3.1416;
uint DataArray[6];

void convertion(const sensor_msgs::JointState::ConstPtr &msg) {
  int i = 0;
  for (i = 0; i < 6; i++) {
    DataArray[i] = uint((msg->position[i]) / PI * 180);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "publish");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("joint_states", 6, convertion);
  ros::Publisher pub =
      n.advertise<std_msgs::Float32MultiArray>("joint_multiarray", 6);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    std_msgs::Float32MultiArray array;
    array.data.clear();

    for (int i = 0; i < 6; i++) {
      array.data.push_back(DataArray[i]);
    }
    pub.publish(array);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
