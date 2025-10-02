#include "../include/rula/ros_communication.h"

ros_communication::ros_communication(int argc, char *argv[], ros::NodeHandle n)
{

   /* FOR COPPELIA */
   pelvis_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/pelvis_pos_topic", 1);
   L5_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/L5_pos_topic", 1);
   L3_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/L3_pos_topic", 1);
   T12_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/T12_pos_topic", 1);
   T8_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/T8_pos_topic", 1);
   neck_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/neck_pos_topic", 1);
   head_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/head_pos_topic", 1);

   right_shoulder_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_shoulder_pos_topic", 1);
   right_upperarm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_upper_arm_pos_topic", 1);
   right_forearm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_forearm_pos_topic", 1);
   right_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_hand_pos_topic", 1);

   left_shoulder_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_shoulder_pos_topic", 1);
   left_upperarm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_upper_arm_pos_topic", 1);
   left_forearm_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_forearm_pos_topic", 1);
   left_hand_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_hand_pos_topic", 1);

   right_upperleg_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_upper_leg_pos_topic", 1);
   right_lowerleg_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_lower_leg_pos_topic", 1);
   right_foot_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_foot_pos_topic", 1);
   right_toe_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_toe_pos_topic", 1);

   left_upperleg_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_upper_leg_pos_topic", 1);
   left_lowerleg_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_lower_leg_pos_topic", 1);
   left_foot_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_foot_pos_topic", 1);
   left_toe_pos_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_toe_pos_topic", 1);

   positions_pub_buffer.push_back(pelvis_pos_pub);
   positions_pub_buffer.push_back(L5_pos_pub);
   positions_pub_buffer.push_back(L3_pos_pub);
   positions_pub_buffer.push_back(T12_pos_pub);
   positions_pub_buffer.push_back(T8_pos_pub);
   positions_pub_buffer.push_back(neck_pos_pub);
   positions_pub_buffer.push_back(head_pos_pub);
   positions_pub_buffer.push_back(right_shoulder_pos_pub);
   positions_pub_buffer.push_back(right_upperarm_pos_pub);
   positions_pub_buffer.push_back(right_forearm_pos_pub);
   positions_pub_buffer.push_back(right_hand_pos_pub);
   positions_pub_buffer.push_back(left_shoulder_pos_pub);
   positions_pub_buffer.push_back(left_upperarm_pos_pub);
   positions_pub_buffer.push_back(left_forearm_pos_pub);
   positions_pub_buffer.push_back(left_hand_pos_pub);
   positions_pub_buffer.push_back(right_upperleg_pos_pub);
   positions_pub_buffer.push_back(right_lowerleg_pos_pub);
   positions_pub_buffer.push_back(right_foot_pos_pub);
   positions_pub_buffer.push_back(right_toe_pos_pub);
   positions_pub_buffer.push_back(left_upperleg_pos_pub);
   positions_pub_buffer.push_back(left_lowerleg_pos_pub);
   positions_pub_buffer.push_back(left_foot_pos_pub);
   positions_pub_buffer.push_back(left_toe_pos_pub);



   /* FOR COPPELIA */
   pelvis_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/pelvis_ori_topic", 1);
   L5_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/L5_ori_topic", 1);
   L3_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/L3_ori_topic", 1);
   T12_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/T12_ori_topic", 1);
   T8_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/T8_ori_topic", 1);
   neck_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/neck_ori_topic", 1);
   head_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/head_ori_topic", 1);

   right_shoulder_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_shoulder_ori_topic", 1);
   right_upperarm_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_upper_arm_ori_topic", 1);
   right_forearm_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_forearm_ori_topic", 1);
   right_hand_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/right_hand_ori_topic", 1);

   left_shoulder_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_shoulder_ori_topic", 1);
   left_upperarm_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_upper_arm_ori_topic", 1);
   left_forearm_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_forearm_ori_topic", 1);
   left_hand_ori_pub = n.advertise<std_msgs::Float32MultiArray>("/vrep/left_hand_ori_topic", 1);

   orientations_pub_buffer.push_back(pelvis_ori_pub);
   orientations_pub_buffer.push_back(L5_ori_pub);
   orientations_pub_buffer.push_back(L3_ori_pub);
   orientations_pub_buffer.push_back(T12_ori_pub);
   orientations_pub_buffer.push_back(T8_ori_pub);
   orientations_pub_buffer.push_back(neck_ori_pub);
   orientations_pub_buffer.push_back(head_ori_pub);
   orientations_pub_buffer.push_back(right_shoulder_ori_pub);
   orientations_pub_buffer.push_back(right_upperarm_ori_pub);
   orientations_pub_buffer.push_back(right_forearm_ori_pub);
   orientations_pub_buffer.push_back(right_hand_ori_pub);
   orientations_pub_buffer.push_back(left_shoulder_ori_pub);
   orientations_pub_buffer.push_back(left_upperarm_ori_pub);
   orientations_pub_buffer.push_back(left_forearm_ori_pub);
   orientations_pub_buffer.push_back(left_hand_ori_pub);


   /* FOR COPPELIA */
   trunk_status_pub = n.advertise<std_msgs::UInt8>("/vrep/trunk_status_topic", 1);
   neck_status_pub = n.advertise<std_msgs::UInt8>("/vrep/neck_status_topic", 1);
   right_upper_arm_status_pub = n.advertise<std_msgs::UInt8>("/vrep/right_upper_arm_status_topic", 1);
   right_lower_arm_status_pub = n.advertise<std_msgs::UInt8>("/vrep/right_lower_arm_status_topic", 1);
   right_wrist_status_pub = n.advertise<std_msgs::UInt8>("/vrep/right_wrist_status_topic", 1);
   left_upper_arm_status_pub = n.advertise<std_msgs::UInt8>("/vrep/left_upper_arm_status_topic", 1);
   left_lower_arm_status_pub = n.advertise<std_msgs::UInt8>("/vrep/left_lower_arm_status_topic", 1);
   left_wrist_status_pub = n.advertise<std_msgs::UInt8>("/vrep/left_wrist_status_topic", 1);


   body_status_pub_buffer.push_back(trunk_status_pub);
   body_status_pub_buffer.push_back(neck_status_pub);
   body_status_pub_buffer.push_back(right_upper_arm_status_pub);
   body_status_pub_buffer.push_back(right_lower_arm_status_pub);
   body_status_pub_buffer.push_back(right_wrist_status_pub);
   body_status_pub_buffer.push_back(left_upper_arm_status_pub);
   body_status_pub_buffer.push_back(left_lower_arm_status_pub);
   body_status_pub_buffer.push_back(left_wrist_status_pub);
}



void ros_communication::ros_publish_positions(vector<vector<float>> positions){

  std_msgs::Float32MultiArray pos_msg;
  vector<float> pos;
  int n_segments = 23;

  ros::Rate r(40);

  for(int segment = 0; segment < n_segments; segment++){
    pos = positions.at(segment);
    pos_msg.data = {pos.at(0),pos.at(1),pos.at(2)}; //pass the x,y,z coordinates to pos_msg
    positions_pub_buffer.at(segment).publish(pos_msg);
  }
  r.sleep();
}


void ros_communication::ros_publish_orientations(vector<vector<float>> orientations){

  std_msgs::Float32MultiArray ori_msg;
  vector<float> ori;
  int n_segments = 15;

  for(int segment = 0; segment < n_segments; segment++){
    ori = orientations.at(segment);
    ori_msg.data = {ori.at(0),ori.at(1),ori.at(2)}; //pass the x,y,z coordinates to ori_msg
    orientations_pub_buffer.at(segment).publish(ori_msg);
  }
}



void ros_communication::ros_publish_status(vector<int> status){

  std_msgs::UInt8 status_msg;

  for(int i = 0; i < 8; i++){
    status_msg.data = status.at(i);
    body_status_pub_buffer.at(i).publish(status_msg);
  }
}
