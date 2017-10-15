#include "AutolivNode.h"
#include "dispatch.h"

namespace octopus
{

// this inline function is to transform uint16t data into int16_t
int16_t inline uint2int(uint16_t data, int bit_num) {return ((int16_t)(data<<(16-bit_num)))>>bit_num;}

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
    sub_can_ = node.subscribe("/can_tx", 1, &AutolivNode::recvCAN, this);

}

AutolivNode::~AutolivNode(){
}

// this function is to get the target format type of a message
int AutolivNode::getTargetType(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgTargetGeneral *ptr = (const MsgTargetGeneral*)msg->msg.data.elems;
    return ptr->type;
}

void AutolivNode::recvCAN(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){

    // deal with target message for now
    if(msg->msg.id > 0x300 && msg->msg.id < 0x309){
        switch(getTargetType(msg)){
            case ID_TARGET_POLAR_SHORT:
                procTargetPolarShort(msg);
            case ID_RAW_POLAR_SHORT:
                procRawPolarShort(msg);
            case ID_TARGET_CARTESIAN:
                procTargetCartesian(msg);
            case ID_TARGET_CARTESIAN_MID:
                procTargetCartesianMid(msg);
            case ID_TARGET_CARTESIAN_MUL:
                procTargetCartesianMul(msg);
            case ID_FREESPACE_SEGMENTS:
                procFreespaceSegments(msg);
            case ID_RAW_POLAR_LONG:
                procRawPolarLong(msg);
            case ID_TARGET_POLAR_LONG:
                procTargetPolarLong(msg);
            default:
                ROS_DEBUG("undefined CAN id!");
            break;
        }
        
    }
}

void AutolivNode::procTargetPolarShort(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgTargetPolarShort *ptr = (const MsgTargetPolarShort*)msg->msg.data.elems;
    float range = (float)(ptr->range_msb << 4 + ptr->range_lsb) / 100;
    float velocity = (float)uint2int(ptr->velocity_msb << 6 + ptr->velocity_lsb, 10) / 10 - 20;
    float bearing = (float)uint2int(ptr->bearing_msb << 8 + ptr->bearing_lsb, 10) / 5;
    uint8_t quality = ptr->quality;
    uint8_t track_id = ptr->track_id;
    uint8_t msg_counter = ptr->msg_counter;
    uint8_t sensor_nr = ptr->sensor_nr;
    uint8_t type = ptr->target_format_type;
    int16_t bearing_obs = uint2int(ptr->bearing_observed_msb << 2 + ptr->bearing_observed_lsb, 6);
    float range_obs = (float)uint2int(ptr->range_observed, 6) / 100;

    // fill message and publish
    TargetPolarShort out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.range = range;
    out.velocity = velocity;
    out.bearing = bearing;
    out.quality = quality;
    out.track_id = track_id;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = target_format_type;
    out.bearing_raw = bearing_obs;
    out.range_raw = range_obs;
    pub_target_polar_short_.publish(out);
}

void AutolivNode::procRawPolarShort(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgRawPolarShort *ptr = (const MsgRawPolarShort*)msg->msg.data.elems;
    float range = (float)(ptr->range_msb << 4 + ptr->range_lsb) / 100;
    float doppler_vel = (float)uint2int(ptr->velocity_msb << 6 + ptr->velocity_lsb, 10) / 10 - 20;
    float bearing = (float)uint2int(ptr->bearing_msb << 8 + ptr->bearing_lsb, 10) / 5; 
    float amp = (float)ptr->amplitude / 2;
    uint8_t msg_counter = ptr->msg_counter;
    uint8_t sensor_nr = ptr->sensor_nr;
    uint8_t type = ptr->target_format_type;
    uint8_t usage = ptr->usage;
    float doppler_alias = (float)ptr->doppler_alias / 5;

    // fill message and publish
    RawPolarShort out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.range = range;
    out.doppler_velocity = doppler_velocity;
    out.bearing = bearing;
    out.amplitude = amp;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = target_format_type;
    out.usage = usage;
    out.doppler_alias = doppler_alias;
    pub_raw_polar_short_.publish(out);
}

void AutolivNode::procTargetCartesian(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    // TODO
}

void AutolivNode::procTargetCartesianMid(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    // TODO
}

void AutolivNode::procTargetCartesianMul(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    // TODO
}

void AutolivNode::procFreespaceSegments(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    // TODO
}

void AutolivNode::procRawPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    // TODO
}

void AutolivNode::procTargetPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    // TODO
}
    
}
