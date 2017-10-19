#ifndef _AUTOLIV_NODE_H_
#define _AUTOLIV_NODE_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "dispatch.h"
#include <dataspeed_can_msgs/CanMessage.h>
#include <dataspeed_can_msgs/CanMessageStamped.h>
#include <autoliv/FreespaceSegments.h>
#include <autoliv/RawPolarLong.h>
#include <autoliv/RawPolarShort.h>
#include <autoliv/TargetCartesian.h>
#include <autoliv/TargetCartesianMid.h>
#include <autoliv/TargetCartesianMul.h>
#include <autoliv/TargetPolarLong.h>
#include <autoliv/TargetPolarShort.h>

namespace octopus{
class AutolivNode{
public:
    AutolivNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
    ~AutolivNode();


private:

    // functions to receive and process the can messages
    int getTargetType(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void recvCAN(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procTargetPolarShort(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procRawPolarShort(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procTargetCartesian(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procTargetCartesianMid(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procTargetCartesianMul(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procFreespaceSegments(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procRawPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);
    void procTargetPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg);

    // functions to send sync and command messages
    MsgSyncMessage* sendSyncMessage(int mode);
    void sendCommand(int sensor_nr, MsgSyncMessage *ptr);
    void sendCommandAll(MsgSyncMessage *ptr);
    // set timer for sending out message
    ros::Timer msg_timer;
    void publishMessageShortLongMode(const ros::TimerEvent& e);
    void publishMessageReset();

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
    ros::Publisher pub_can_;

};
}

#endif // _AUTOLIV_NODE_H_


