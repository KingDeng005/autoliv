#include "AutolivNode.h"
#include "dispatch.h"

namespace octopus
{

// this inline function is to transform uint16t data into int16_t
int16_t inline uint2int(uint16_t data, int bit_num) {return ((int16_t)(data<<(16-bit_num)))>>(16-bit_num);}

uint8_t inline crc8(MsgSyncMessage *ptr){
    crc = *ptr;
    for(unsigned short i = 8; i > 0; --i){
        if (crc & 0x80){
            crc = (crc << 1) ^ 0x31;}
        else{
            crc = (crc << 1);
        }
    }
    return (crc);
}

AutolivNode::AutolivNode(ros::NodeHandle &node, ros::NodeHandle &priv_nh){

    // publisher
    pub_target_polar_short_ = node.advertise<autoliv::TargetPolarShort>("target_polar_short", 100);
    pub_raw_polar_short_ = node.advertise<autoliv::RawPolarShort>("raw_polar_short", 2);
    pub_target_cartesian_ = node.advertise<autoliv::TargetCartesian>("target_cartesian", 2);
    pub_target_cartesian_mid_ = node.advertise<autoliv::TargetCartesianMid>("target_cartesian_mid", 2);
    pub_target_cartesian_mul_ = node.advertise<autoliv::TargetCartesianMul>("target_cartesian_mul", 2);
    pub_freespace_segments_ = node.advertise<autoliv::FreespaceSegments>("freespace_segments", 2);
    pub_raw_polar_long_ = node.advertise<autoliv::RawPolarLong>("raw_polar_long", 2);
    pub_target_polar_long_ = node.advertise<autoliv::TargetPolarLong>("target_polar_long", 2);
    pub_can_ = node.advertise<dataspeed_can_msgs::CanMessage>("/can_tx", 100);

    // subscriber
    sub_can_ = node.subscribe("/can_tx", 1, &AutolivNode::recvCAN, this);

}

AutolivNode::~AutolivNode(){
}

void AutolivNode::sendSyncMessage(int mode){
    dataspeed_can_msgs::CanMessage out;
    out.id = ID_SYNC_MESSAGE;
    out.extended = false;
    out.dlc = sizeof(MsgSyncMessage);

    MsgSyncMessage *ptr = (MsgSyncMessage*)out.data.elems;
    memset(ptr, 0x00, sizeof(*ptr));

    ptr->sensor_1_mode = (uint8_t)mode;
    ptr->sensor_2_mode = (uint8_t)mode;
    ptr->sensor_3_mode = (uint8_t)mode;
    ptr->sensor_4_mode = (uint8_t)mode;
    ptr->msg_counter = 0x20;
    ptr->data_channel_msb = 0x00;
    ptr->data_channel_lsb = 0x00;
    if(mode == MODE_SENSOR_RESET){
        ptr->byte_1 = 0x00;
        ptr->byte_2 = 0x00;
        ptr->byte_3 = 0x00;
    }
    else{
        ptr->byte_1 = 0xFF;
        ptr->byte_2 = 0xFF;
        ptr->byte_3 = 0xFF;
    }
    pub_can_.publish(out);  
}

void AutolivNode::sendCommand(int sensor_nr){
    dataspeed_can_msgs::CanMessage out;
    out.id = 0x200 + sensor_nr;
    out.dlc = 7;

    MsgCommandMessage *ptr = (MsgCommandMessage*)out.data.elems;
    memset(ptr, 0x00, sizeof(*ptr));

    ptr->msg_counter = 0x0;
    ptr->meas_page_select = 0x2;
    ptr->data_channel_1_msb = 0x00;
    ptr->data_channel_1_lsb = 0x00;
    ptr->data_channel_2_msb = 0x00;
    ptr->data_channel_2_lsb = 0x00;
    ptr->sync_msg_content = crc8(MsgSyncMessage *ptr);
    pub_can_.publish(out);

}

void AutolivNode::sendCommandAll(){
    for(int i = 1; i <= 4; ++i){
        sendCommand(i);
    }
}

// this function is to get the target format type of a message
int AutolivNode::getTargetType(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgTargetGeneral *ptr = (const MsgTargetGeneral*)msg->msg.data.elems;
    return ptr->type;
}

void AutolivNode::recvCAN(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){

    // deal with target message for now
    if(msg->msg.id > ID_TARGET_LOWER && msg->msg.id < ID_TARGET_UPPER){
        switch(getTargetType(msg)){
            case TYPE_TARGET_POLAR_SHORT:
                procTargetPolarShort(msg);
            case TYPE_RAW_POLAR_SHORT:
                procRawPolarShort(msg);
            case TYPE_TARGET_CARTESIAN:
                procTargetCartesian(msg);
            case TYPE_TARGET_CARTESIAN_MID:
                procTargetCartesianMid(msg);
            case TYPE_TARGET_CARTESIAN_MUL:
                procTargetCartesianMul(msg);
            case TYPE_FREESPACE_SEGMENTS:
                procFreespaceSegments(msg);
            case TYPE_RAW_POLAR_LONG:
                procRawPolarLong(msg);
            case TYPE_TARGET_POLAR_LONG:
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
    autoliv::TargetPolarShort out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.range = range;
    out.velocity = velocity;
    out.bearing = bearing;
    out.quality = quality;
    out.track_id = track_id;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = type;
    out.bearing_raw = bearing_obs;
    out.range_raw = range_obs;
    pub_target_polar_short_.publish(out);
}

void AutolivNode::procRawPolarShort(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgRawPolarShort *ptr = (const MsgRawPolarShort*)msg->msg.data.elems;
    float range = (float)(ptr->range_msb << 4 + ptr->range_lsb) / 100;
    float doppler_vel = (float)uint2int(ptr->doppler_velocity_msb << 6 + ptr->doppler_velocity_lsb, 10) / 10 - 20;
    float bearing = (float)uint2int(ptr->bearing_msb << 8 + ptr->bearing_lsb, 10) / 5; 
    float amp = (float)ptr->amplitude / 2;
    uint8_t msg_counter = ptr->msg_counter;
    uint8_t sensor_nr = ptr->sensor_nr;
    uint8_t type = ptr->target_format_type;
    uint8_t usage = ptr->usage;
    float doppler_alias = (float)ptr->doppler_alias / 5;

    // fill message and publish
    autoliv::RawPolarShort out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.range = range;
    out.doppler_velocity = doppler_vel;
    out.bearing = bearing;
    out.amplitude = amp;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = type;
    out.usage = usage;
    out.doppler_alias = doppler_alias;
    pub_raw_polar_short_.publish(out);
}

void AutolivNode::procTargetCartesian(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgTargetCartesian *ptr = (const MsgTargetCartesian*)msg->msg.data.elems;
    float distance_x = (float)uint2int(ptr->distance_x_msb << 3 + ptr->distance_x_lsb, 11) / 50;
    float velocity_x = (float)uint2int(ptr->velocity_x_msb << 4 + ptr->velocity_x_lsb, 9) / 10;
    float velocity_y = (float)uint2int(ptr->velocity_y_msb << 5 + ptr->velocity_y_lsb, 9) / 10;
    float distance_y = (float)uint2int(ptr->distance_y_msb << 8 + ptr->distance_y_lsb, 11) / 50;
    uint8_t msg_counter = ptr->msg_counter;
    uint8_t sensor_nr = ptr->sensor_nr;
    uint8_t type = ptr->target_format_type;
    uint8_t obj_type = ptr->obj_type;
    uint8_t quality = ptr->quality;
    uint8_t track_id = ptr->track_id;

    // fill message and publish
    autoliv::TargetCartesian out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.distance_x = distance_x;
    out.velocity_x = velocity_x;
    out.velocity_y = velocity_y;
    out.distance_y = distance_y;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = type;
    out.object_type = obj_type;
    out.quality = quality;
    out.track_id = track_id;
    pub_target_cartesian_.publish(out);
}

void AutolivNode::procTargetCartesianMid(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgTargetCartesianMid *ptr = (const MsgTargetCartesianMid*)msg->msg.data.elems;
    float distance_x = (float)uint2int(ptr->distance_x_msb << 4 + ptr->distance_x_lsb, 12) / 20;
    uint8_t track_id = ptr->track_id;
    float velocity_x = (float)uint2int(ptr->velocity_x_msb << 2 + ptr->velocity_x_lsb, 10) / 10 - 20;
    float velocity_y = (float)uint2int(ptr->velocity_y_msb << 4 + ptr->velocity_y_lsb, 10) / 10;
    uint8_t quality = ptr->quality;
    uint8_t msg_counter = ptr->msg_counter;
    uint8_t sensor_nr = ptr->sensor_nr;
    uint8_t type = ptr->target_format_type;
    float distance_y = (float)uint2int(ptr->distance_y_msb << 8 + ptr->distance_y_lsb, 12) / 50;

    // fill message and publish
    autoliv::TargetCartesianMid out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.distance_x = distance_x;
    out.track_id = track_id;
    out.velocity_x = velocity_x;
    out.velocity_y = velocity_y;
    out.quality = quality;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = type;
    out.distance_y = distance_y;
    pub_target_cartesian_mid_.publish(out);
}

void AutolivNode::procTargetCartesianMul(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgTargetCartesianMul *ptr = (const MsgTargetCartesianMul*)msg->msg.data.elems;
    float distance_x = (float)(ptr->distance_x_msb << 3 + ptr->distance_x_lsb) / 20;
    uint8_t track_id = ptr->track_id;
    float velocity_x = (float)uint2int(ptr->velocity_x_msb << 2 + ptr->velocity_x_lsb, 10) / 10 - 20;
    float velocity_y = (float)uint2int(ptr->velocity_y_msb << 4 + ptr->velocity_y_lsb, 10) / 10;
    uint8_t scan_type = ptr->scan_type;
    uint8_t quality = ptr->quality;
    uint8_t msg_counter = ptr->msg_counter;
    uint8_t sensor_nr = ptr->sensor_nr;
    uint8_t type = ptr->target_format_type;
    float distance_y = (float)uint2int(ptr->distance_y_msb << 8 + ptr->distance_y_lsb, 12) / 50;

    // fill message and publish
    autoliv::TargetCartesianMul out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.distance_x = distance_x;
    out.track_id = track_id;
    out.velocity_x = velocity_x;
    out.velocity_y = velocity_y;
    out.scan_type = scan_type;
    out.quality = quality;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = type;
    out.distance_y = distance_y;
    pub_target_cartesian_mul_.publish(out);
}

void AutolivNode::procFreespaceSegments(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgFreespaceSegments *ptr = (const MsgFreespaceSegments*)msg->msg.data.elems;
    float seg_0 = (float)(ptr->seg_0) * .8;
    float seg_1 = (float)(ptr->seg_1_msb << 6 + ptr->seg_1_lsb) * .8;
    float seg_2 = (float)(ptr->seg_2_msb << 5 + ptr->seg_2_lsb) * .8;
    float seg_3 = (float)(ptr->seg_3_msb << 4 + ptr->seg_3_lsb) * .8;
    float seg_4 = (float)(ptr->seg_4_msb << 3 + ptr->seg_4_lsb) * .8;
    float seg_5 = (float)(ptr->seg_1_msb << 2 + ptr->seg_5_lsb) * .8;
    uint8_t msg_counter = ptr->msg_counter;
    uint8_t sensor_nr = ptr->sensor_nr;
    uint8_t type = ptr->target_format_type;
    float seg_6 = (float)(ptr->seg_6_msb << 5 + ptr->seg_6_lsb) * .8;
    uint8_t seg_idx = ptr->seg_idx;

    // fill message and publish
    autoliv::FreespaceSegments out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.segment0 = seg_0;
    out.segment1 = seg_1;
    out.segment2 = seg_2;
    out.segment3 = seg_3;
    out.segment4 = seg_4;
    out.segment5 = seg_5;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = type;
    out.segment6 = seg_6;
    out.segment_index = seg_idx;
    pub_freespace_segments_.publish(out);
}

void AutolivNode::procRawPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgRawPolarLong *ptr = (const MsgRawPolarLong*)msg->msg.data.elems;
    float range = (float)(ptr->range_msb << 4 + ptr->range_lsb) / 20;
    float doppler_vel = (float)uint2int(ptr->doppler_velocity_msb << 6 + ptr->doppler_velocity_lsb, 10) / 10 - 20;
    float bearing = (float)uint2int(ptr->bearing_msb << 8 + ptr->bearing_lsb, 10) / 5; 
    float amp = (float)ptr->amplitude / 2;
    uint8_t msg_counter = ptr->msg_counter;
    uint8_t sensor_nr = ptr->sensor_nr;
    uint8_t type = ptr->target_format_type;
    uint8_t usage = ptr->usage;
    float doppler_alias = (float)ptr->doppler_alias / 5;

    // fill message and publish
    autoliv::RawPolarLong out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.range = range;
    out.doppler_velocity = doppler_vel;
    out.bearing = bearing;
    out.amplitude = amp;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = type;
    out.usage = usage;
    out.doppler_alias = doppler_alias;
    pub_raw_polar_long_.publish(out);
}

void AutolivNode::procTargetPolarLong(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg){
    const MsgTargetPolarLong *ptr = (const MsgTargetPolarLong*)msg->msg.data.elems;
    float range = (float)(ptr->range_msb << 4 + ptr->range_lsb) / 20;
    float velocity = (float)uint2int(ptr->velocity_msb << 6 + ptr->velocity_lsb, 10) / 10 - 20;
    float bearing = (float)uint2int(ptr->bearing_msb << 8 + ptr->bearing_lsb, 10) / 5;
    uint8_t quality = ptr->quality;
    uint8_t track_id = ptr->track_id;
    uint8_t msg_counter = ptr->msg_counter;
    uint8_t sensor_nr = ptr->sensor_nr;
    uint8_t type = ptr->target_format_type;
    uint8_t obj_type = ptr->obj_type;

    // fill message and publish
    autoliv::TargetPolarLong out;
    out.header.frame_id = "base_link";
    out.header.stamp = ros::Time::now();
    out.range = range;
    out.velocity = velocity;
    out.bearing = bearing;
    out.quality = quality;
    out.track_id = track_id;
    out.msg_counter = msg_counter;
    out.sensor_nr = sensor_nr;
    out.target_format_type = type;
    out.object_type = obj_type;
    pub_target_polar_long_.publish(out);
}
    
}
