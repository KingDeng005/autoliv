#include "autoliv_flr_driver_node.h"
#include "dispatch.h"

namespace octopus
{

int inline id2topic(int id)
{
  int topic_index = 10;
  switch(id)
  {
    case ID_VIDEO_OBJECT_0_REPORT_A:  topic_index = 0; break;
    case ID_VIDEO_OBJECT_1_REPORT_A:  topic_index = 1; break;
    case ID_VIDEO_OBJECT_2_REPORT_A:  topic_index = 2; break;
    case ID_VIDEO_OBJECT_3_REPORT_A:  topic_index = 3; break;
    case ID_VIDEO_OBJECT_4_REPORT_A:  topic_index = 4; break;
    case ID_VIDEO_OBJECT_5_REPORT_A:  topic_index = 5; break;
    case ID_VIDEO_OBJECT_6_REPORT_A:  topic_index = 6; break;
    case ID_VIDEO_OBJECT_7_REPORT_A:  topic_index = 7; break;
    case ID_VIDEO_OBJECT_8_REPORT_A:  topic_index = 8; break;
    case ID_VIDEO_OBJECT_9_REPORT_A:  topic_index = 9; break;
    case ID_VIDEO_OBJECT_0_REPORT_B:  topic_index = 0; break;
    case ID_VIDEO_OBJECT_1_REPORT_B:  topic_index = 1; break;
    case ID_VIDEO_OBJECT_2_REPORT_B:  topic_index = 2; break;
    case ID_VIDEO_OBJECT_3_REPORT_B:  topic_index = 3; break;
    case ID_VIDEO_OBJECT_4_REPORT_B:  topic_index = 4; break;
    case ID_VIDEO_OBJECT_5_REPORT_B:  topic_index = 5; break;
    case ID_VIDEO_OBJECT_6_REPORT_B:  topic_index = 6; break;
    case ID_VIDEO_OBJECT_7_REPORT_B:  topic_index = 7; break;
    case ID_VIDEO_OBJECT_8_REPORT_B:  topic_index = 8; break;
    case ID_VIDEO_OBJECT_9_REPORT_B:  topic_index = 9; break;
    default:  topic_index = 10; break;
  }
  return topic_index;
}

autoliv_flr_driver_node::~autoliv_flr_driver_node()
{

}

autoliv_flr_driver_node::autoliv_flr_driver_node(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
  : hb_(node, priv_nh, "wingman_flc20_node")
{

  pub_video_objects_ = node.advertise<bendix_flc20_msgs::VideoObjectReport>("bendix/flc20", 100);
  sub_can_ = node.subscribe("/can_rx", 1, &Flc20DriverNode::recvCAN, this);

  for(int i=0; i<9; i++)
    object[i].completion = 0;

}


void Flc20DriverNode::procVideoObjectReportA(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg)
{
  const MsgObjectReportA *ptr = (const MsgObjectReportA*)msg->msg.data.elems;

  // example:
  object[id2topic(msg->msg.id)].data.camera_id = ptr->object_id;
  // renyuan:add code here
  
  double a = ptr->longitude_distance_lsb;
  double b = ptr->longitude_distance_msb;
  object[id2topic(msg->msg.id)].data.longitude_distance = .05*(a+b*256);

  a = ptr->relative_velocity_lsb;
  b = ptr->relative_velocity_msb;
  object[id2topic(msg->msg.id)].data.relative_velocity = .05*(a+b*256);

  a = ptr->tan_left_angle_lsb;
  b = ptr->tan_left_angle_msb;
  object[id2topic(msg->msg.id)].data.tan_left_angle = .0002*(a+b*256);
  
  a = ptr->tan_right_angle_lsb;
  b = ptr->tan_right_angle_msb;
  object[id2topic(msg->msg.id)].data.tan_right_angle = .0002*(a+b*256);

  object[id2topic(msg->msg.id)].data.message_counter = ptr->message_counter;
  



  if(object[id2topic(msg->msg.id)].completion == 1)
  {
    pub_video_objects_.publish(object[id2topic(msg->msg.id)]);
    object[id2topic(msg->msg.id)].completion = 0;
  }
  else
    object[id2topic(msg->msg.id)].completion += 1;
  
  
}

void Flc20DriverNode::procVideoObjectReportB(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg)
{
  const MsgObjectReportB *ptr = (const MsgObjectReportB*)msg->msg.data.elems;

  //renyuan:add code here
  object[id2topic(msg->msg.id)].data.lane = ptr->lane_position;


  if(object[id2topic(msg->msg.id)].completion == 1)
  {
    pub_video_objects_.publish(object[id2topic(msg->msg.id)]);
    object[id2topic(msg->msg.id)].completion = 0;
  }
  else
    object[id2topic(msg->msg.id)].completion += 1;
}

void Flc20DriverNode::recvCAN(const dataspeed_can_msgs::CanMessageStamped::ConstPtr &msg)
{
  switch (msg->msg.id) 
  {
    case ID_VIDEO_OBJECT_0_REPORT_A:  
    case ID_VIDEO_OBJECT_1_REPORT_A:  
    case ID_VIDEO_OBJECT_2_REPORT_A:  
    case ID_VIDEO_OBJECT_3_REPORT_A:  
    case ID_VIDEO_OBJECT_4_REPORT_A:  
    case ID_VIDEO_OBJECT_5_REPORT_A:  
    case ID_VIDEO_OBJECT_6_REPORT_A:  
    case ID_VIDEO_OBJECT_7_REPORT_A:
    case ID_VIDEO_OBJECT_8_REPORT_A:
    case ID_VIDEO_OBJECT_9_REPORT_A:  
      procVideoObjectReportA(msg);
      break;
    case ID_VIDEO_OBJECT_0_REPORT_B:  
    case ID_VIDEO_OBJECT_1_REPORT_B:  
    case ID_VIDEO_OBJECT_2_REPORT_B:  
    case ID_VIDEO_OBJECT_3_REPORT_B:  
    case ID_VIDEO_OBJECT_4_REPORT_B:  
    case ID_VIDEO_OBJECT_5_REPORT_B:
    case ID_VIDEO_OBJECT_6_REPORT_B:
    case ID_VIDEO_OBJECT_7_REPORT_B:
    case ID_VIDEO_OBJECT_8_REPORT_B:
    case ID_VIDEO_OBJECT_9_REPORT_B:  
      procVideoObjectReportB(msg);
      break;
    default:
      ROS_WARN("Unidentified CAN ID. Ignore");
  }
}




}
