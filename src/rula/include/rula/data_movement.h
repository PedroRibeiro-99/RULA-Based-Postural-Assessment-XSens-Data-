#ifndef DATA_MOVEMENT_H
#define DATA_MOVEMENT_H

#include <math.h>
#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "data_files.h"
#include "ros_communication.h"
#include "rula.h"

using namespace std;
using namespace Eigen;

typedef struct Position{
  string segment;
  int frame;
  float x;
  float y;
  float z;
}position;

typedef struct Orientation_quat{
  string segment;
  int frame;
  float q0;
  float q1;
  float q2;
  float q3;
}orientation_quaternion;

typedef struct Orientation_deg{
  string segment;
  int frame;
  float x;
  float y;
  float z;
}orientation_deg;

typedef struct JointAngleZXY{
  string joint;
  int frame;
  double z;
  double x;
  double y;
}joint_angle_zxy;

typedef struct ErgonomicJointAngleZXY{
  string ergonomicJoint;
  int frame;
  double z;
  double x;
  double y;
}ergonomicJoint_angle_zxy;



class XsensData{
private:
  int frames;
  vector<string> joints_names;
  vector<string> ergonomicJoints_names;
  vector<string> segments_names;
  vector<vector<position>> positions;
  vector<vector<orientation_quaternion>> orientations_quat;
  vector<vector<orientation_deg>> orientations_deg;
  vector<vector<joint_angle_zxy>> joints_angles_zxy;
  vector<vector<ergonomicJoint_angle_zxy>> ergonomicJoints_angles_zxy;
  vector<Rula> rula_buffer;
  vector<int> rula_scores;

public:
  XsensData();
  //obtain the number of frames recorded from the xml extracted data
  void frames_extract(XsensXml xsensXml);

  //obtain the joints names from the xml extracted data
  void joints_names_extract(XsensXml xsensXml);

  void ergonomicJoints_names_extract(XsensXml xsensXml);

  //obtain the segments names from the xml extracted data
  void segments_names_extract(XsensXml xsensXml);

  //obtain and handle the joints values from the XML extracted data
  void jointAnglesZXY_extract(XsensXml xsensXml);

  void ergonomicJointAnglesZXY_extract(XsensXml xsensXml);

  //obtain and handle the positions values from the XML extracted data
  void positions_extract(XsensXml xsensXml);

  //obtain and handle the orientations values from the XML extracted data
  void orientations_extract(XsensXml xsensXml);

  void convert_orientations_quat2deg();

  void rotate_orientations(string segment_name, vector<float> rotation);

  int get_frames_number();

  void get_segments_name(vector<string> &segments_name);


  //to get the information about a specific joint in a specific frame recorded
  int get_jointangleZXY(int frame, string joint_name, joint_angle_zxy &joint_obj);

  int get_ergonomicJointangleZXY(int frame, string joint_name, ergonomicJoint_angle_zxy &ergonomicJoint_obj);

  //to get the information about the position of a specific segment in a specific frame recorded
  int get_segment_position(int frame, string segment_name, position &pos_obj);

  int get_frame_positions(int frame, vector<position> &pos_obj);

  //to get the information about the orientation of a specific segment in a specific frame recorded
  int get_segment_orientation_quaternion(int frame, string segment_name, orientation_quaternion &or_obj);

  int get_segment_orientation_deg(int frame, string segment_name, orientation_deg &or_obj);

  void rula_evaluation(string arm_side);

  void publish_frame_positions(ros_communication ros_handler, int frame);

  void publish_frame_orientationsDeg(ros_communication ros_handler, int frame);

  void publish_frame_rula_status(ros_communication ros_handler, int frame, string side);

  void publish_data(ros_communication ros_handler,string side);

  void sort_rula_evaluation(vector<Rula> &sorted_rula);

  void write_data_files(string filename, string arm_side);
};

#endif // DATA_MOVEMENT_H
