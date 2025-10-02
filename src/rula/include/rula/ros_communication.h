#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

//#include "data_movement.h"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt8.h"
#include <vector>

using namespace std;

class ros_communication
{
private:
  //ros::NodeHandle n;
  ros::Publisher pelvis_pos_pub;
  ros::Publisher L5_pos_pub;
  ros::Publisher L3_pos_pub;
  ros::Publisher T12_pos_pub;
  ros::Publisher T8_pos_pub;
  ros::Publisher neck_pos_pub;
  ros::Publisher head_pos_pub;

  ros::Publisher right_shoulder_pos_pub;
  ros::Publisher right_upperarm_pos_pub;
  ros::Publisher right_forearm_pos_pub;
  ros::Publisher right_hand_pos_pub;

  ros::Publisher left_shoulder_pos_pub;
  ros::Publisher left_upperarm_pos_pub;
  ros::Publisher left_forearm_pos_pub;
  ros::Publisher left_hand_pos_pub;

  ros::Publisher  right_upperleg_pos_pub;
  ros::Publisher  right_lowerleg_pos_pub;
  ros::Publisher  right_foot_pos_pub;
  ros::Publisher  right_toe_pos_pub;
  ros::Publisher  left_upperleg_pos_pub;
  ros::Publisher  left_lowerleg_pos_pub;
  ros::Publisher  left_foot_pos_pub;
  ros::Publisher  left_toe_pos_pub;


  ros::Publisher pelvis_ori_pub;
  ros::Publisher L5_ori_pub;
  ros::Publisher L3_ori_pub;
  ros::Publisher T12_ori_pub;
  ros::Publisher T8_ori_pub;
  ros::Publisher neck_ori_pub;
  ros::Publisher head_ori_pub;

  ros::Publisher right_shoulder_ori_pub;
  ros::Publisher right_upperarm_ori_pub;
  ros::Publisher right_forearm_ori_pub;
  ros::Publisher right_hand_ori_pub;

  ros::Publisher left_shoulder_ori_pub;
  ros::Publisher left_upperarm_ori_pub;
  ros::Publisher left_forearm_ori_pub;
  ros::Publisher left_hand_ori_pub;



  ros::Publisher right_upper_arm_status_pub;
  ros::Publisher right_lower_arm_status_pub;
  ros::Publisher right_wrist_status_pub;
  ros::Publisher left_upper_arm_status_pub;
  ros::Publisher left_lower_arm_status_pub;
  ros::Publisher left_wrist_status_pub;
  ros::Publisher neck_status_pub;
  ros::Publisher trunk_status_pub;

  vector<ros::Publisher> positions_pub_buffer;
  vector<ros::Publisher> orientations_pub_buffer;
  vector<ros::Publisher> body_status_pub_buffer;

public:
  ros_communication(int argc, char *argv[],ros::NodeHandle n);
 // void ros_publish_data(XsensData xsens_data);
  void ros_publish_positions(vector<vector<float>> positions);
  void ros_publish_orientations(vector<vector<float>> orientations);
  void ros_publish_status(vector<int> status);

};

#endif // ROS_COMMUNICATION_H
