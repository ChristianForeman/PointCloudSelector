#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "node_name");
  ros::NodeHandle n;
  ros::spin();
  return 0;
}