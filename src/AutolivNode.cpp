#include "AutolivNode.h"
#include "dispatch.h"

namespace octopus
{
AutolivNode::~AutolivNode(){

}
AutolivNode::AutolivNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh){

    // publisher
	pub_target_polar_short_ = node.advertise<TargetPolarShort>('target_polar_short', 100);
	pub_raw_polar_short_ = node.advertise<RawPolarShort>('raw_polar_short', 2);
	pub_target_cartesian_ = node.advertise<TargetCartesian>('target_cartesian', 2);
	pub_target_cartesian_mid = node.advertise<TargetCartesianMid>('target_cartesian_mid', 2);
	pub_target_cartesian_mul = node.advertise<TargetCartesianMul>('target_cartesian_mul', 2);
	pub_freespace_segments = node.advertise<FreespaceSegments>('freespace_segments', 2);
	pub_raw_polar_long = node.advertise<RawPolarLong>('raw_polar_long', 2);
	pub_target_polar_long = node.advertise<TargetPolarLong>('target_polar_long', 2);

    // subscriber
    sub_can_ = node.subscribe("/can_tx", 1, &AutolivNode::recvCAN,this);

}

void procTargetPolarShort(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
	const TargetPolarShort *ptr = (const TargetPolarShort*)msg->msg.data.elems;
	
}

void procRawPolarShort(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){

}

void procTargetCartesian(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){

}

void procTargetCartesianMid(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){

}

void procTargetCartesianMul(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){

}

void procFreespaceSegments(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){

}

void procRawPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){

}

void procTargetPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){

}

	
}
