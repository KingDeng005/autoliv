#include <ros/ros.h>
#include "Flc20DriverNode.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wingman_flc20");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  octopus::Flc20DriverNode n(node, priv_nh);
  ros::spin();

  return 0;
}