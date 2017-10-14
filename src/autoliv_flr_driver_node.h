#ifndef _autoliv_flr_driver_node_h
#define _autoliv_flr_driver_node_h

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <dataspeed_can_msgs/CanMessage.h>
#include <dataspeed_can_msgs/CanMessageStamped.h>
#include <bendix_flc20_msgs/VideoObjectReport.h>

#include <heartbeat_sender/heartbeat_sender.h>

namespace octopus
{
class autoliv_flr_driver_node
{
public:
  autoliv_flr_driver_node(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
  ~autoliv_flr_driver_node();

private:
  void recvCAN(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
  void procVideoObjectReportA(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
  void procVideoObjectReportB(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);

  ros::Subscriber sub_can_;
  ros::Publisher pub_video_objects_;
  octopus_monitor::HeartbeatSender hb_;

  autoliv_flr_msgs::VideoObjectReport object[11];
};
}
#endif
