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

class AutolivNode
{
public:
	AutolivNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
	~AutolivNode();

private:
	// functions to receive and process the can messages
	void recvCAN(const dataspeed_can_msgs::CamMessagesStamped::ConstPtr &msg);
	void procTargetPolarShort(const TargetPolarShort::ConstPtr &msg);
	void procRawPolarShort(const RawPolarShort::ConstPtr &msg);
	void procTargetCartesian(const TargetCartesian::ConstPtr &msg);
	void procTargetCartesianMid(const TargetCartesianMid::ConstPtr &msg);
	void procTargetCartesianMul(const TargetCartesianMul::ConstPtr &msg);
	void procFreespaceSegments(const FreespaceSegments::ConstPtr &msg);
	void procRawPolarLong(const RawPolarLong::ConstPtr &msg);
	void procTargetPolarLong(const TargetPolarLong::Constr &msg);

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

#endif // _AUTOLIV_NODE_H_


