#ifndef _AUTOLIV_NODE_H_
#define _AUTOLIV_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <dataspeed_can_msgs/CanMessageStamped.h>
#include <FreespaceSegments.h>
#include <RawPolarLong.h>
#include <RawPolarShort.h>
#include <TargetCartesian.h>
#include <TargetCartesianMid.h>
#include <TargetCartesianMul.h>
#include <TargetPolarLong.h>
#include <TargetPolarShort.h>

namespace octopus{
class AutolivNode{
public:
    AutolivNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
    ~AutolivNode();

private:

    // functions to receive and process the can messages
    void recvCAN(const dataspeed_can_msgs::CamMessagesStamped::ConstPtr &msg);
    void procTargetPolarShort(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procRawPolarShort(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procTargetCartesian(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procTargetCartesianMid(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procTargetCartesianMul(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procFreespaceSegments(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procRawPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procTargetPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);

    // subscriber
    ros::Subscriber sub_can_;

    // publiser
    ros::Publisher pub_target_polar_short_;
    ros::Publisher pub_raw_polar_short_;
    ros::Publisher pub_target_cartesian_;
    ros::Publisher pub_target_cartesian_mid_;
    ros::Publisher pub_target_cartesian_mul_;
    ros::Publisher pub_freespace_segments_;
    ros::Publisher pub_raw_polar_long_;
    ros::Publisher pub_target_polar_long_;

};
}

#endif // _AUTOLIV_NODE_H_


