#include "../include/rula/data_movement.h"
#include <QDateTime>
#include <QDate>
#include <QTime>
#include <cctype>

XsensData::XsensData(){}

void XsensData::frames_extract(XsensXml xsensXml){
  this->frames = xsensXml.get_frames();
}

void XsensData::joints_names_extract(XsensXml xsensXml){
  xsensXml.get_joints_names(this->joints_names);
}

void XsensData::ergonomicJoints_names_extract(XsensXml xsensXml){
  xsensXml.get_ergonomicJoints_names(this->ergonomicJoints_names);
}

void XsensData::segments_names_extract(XsensXml xsensXml){
  xsensXml.get_segments_names(this->segments_names);
}

void XsensData::jointAnglesZXY_extract(XsensXml xsensXml){
  vector<vector<double>> jointsAnglesValues;
  vector<string> jointAnglesStr;

  //Get the vector of strings containing all the joints angles values in each frame
  xsensXml.get_jointAngles(jointAnglesStr);
  //Parse and Extract the values from each string containing all the joints angles values in each frame to a bidimensional vector of double
  //In this new vector the index represent each frame of the movement
  xsensXml.data_parsing(jointAnglesStr, jointsAnglesValues);

  joint_angle_zxy joint_obj;
  vector<joint_angle_zxy> joint_obj_vec;

  for(int frame=0; frame<this->frames; frame++){//check all the frames contained in
  joint_obj_vec.clear();
    for (int n_joint=0; n_joint<(jointsAnglesValues.at(frame).size())/3; n_joint++){  //each joint is represented by 3 axis (x,y and z). The string of each frame contains the representation of each joint in the 3 axis
      joint_obj.frame = frame;
      joint_obj.z = jointsAnglesValues[frame][3*n_joint];
      joint_obj.x = jointsAnglesValues[frame][3*n_joint+1];
      joint_obj.y = jointsAnglesValues[frame][3*n_joint+2];
      joint_obj.joint = joints_names.at(n_joint);
      joint_obj_vec.push_back(joint_obj);
    }
    joints_angles_zxy.push_back(joint_obj_vec);
  }
}


void XsensData::ergonomicJointAnglesZXY_extract(XsensXml xsensXml){
  vector<vector<double>> ergonomicJointsAnglesValues;
  vector<string> ergonomicJointAnglesStr;

  //Get the vector of strings containing all the ergonomic joints angles values in each frame
  xsensXml.get_ergonomicJointAngles(ergonomicJointAnglesStr);
  //Parse and Extract the values from each string containing all the ergonomic joints angles values in each frame to a bidimensional vector of double
  //In this new vector the index represent each frame of the movement
  xsensXml.data_parsing(ergonomicJointAnglesStr, ergonomicJointsAnglesValues);

  ergonomicJoint_angle_zxy ergonomicJoint_obj;
  vector<ergonomicJoint_angle_zxy> ergonomicJoint_obj_vec;

  for(int frame=0; frame<this->frames; frame++){//check all the frames contained in
  ergonomicJoint_obj_vec.clear();
    for (int n_joint=0; n_joint<(ergonomicJointsAnglesValues.at(frame).size())/3; n_joint++){  //each joint is represented by 3 axis (x,y and z). The string of each frame contains the representation of each joint in the 3 axis
      ergonomicJoint_obj.frame = frame;
      ergonomicJoint_obj.z = ergonomicJointsAnglesValues[frame][3*n_joint];
      ergonomicJoint_obj.x = ergonomicJointsAnglesValues[frame][3*n_joint+1];
      ergonomicJoint_obj.y = ergonomicJointsAnglesValues[frame][3*n_joint+2];
      ergonomicJoint_obj.ergonomicJoint = ergonomicJoints_names.at(n_joint);
      ergonomicJoint_obj_vec.push_back(ergonomicJoint_obj);
    }
    ergonomicJoints_angles_zxy.push_back(ergonomicJoint_obj_vec);
  }
}


//obtain and handle the positions values from the XML extracted data
void XsensData::positions_extract(XsensXml xsensXml){
  vector<vector<double>> positionValues;
  vector<string> positionStr;

  //Get the vector of strings containing all the position values in each frame
  xsensXml.get_positions(positionStr);
  //Parse and Extract the values from each string containing all the position values in each frame to a bidimensional vector of double
  //In this new vector the index represent each frame of the movement
  xsensXml.data_parsing(positionStr, positionValues);

  position pos_obj;
  vector<position> pos_obj_vec;

  for(int frame=0; frame<this->frames; frame++){
  pos_obj_vec.clear();
    for (int n_segment=0; n_segment<(positionValues.at(frame).size())/3; n_segment++){  //each segment position is represented by 3 axis (x,y and z). The string of each frame contains the position representation of each segment in the 3 axis
      pos_obj.frame = frame;
      pos_obj.x = positionValues[frame][3*n_segment];
      pos_obj.y = positionValues[frame][3*n_segment+1];
      pos_obj.z = positionValues[frame][3*n_segment+2];
      pos_obj.z = pos_obj.z - 0.2758;
      pos_obj.segment = segments_names.at(n_segment);
      pos_obj_vec.push_back(pos_obj);
    }
    positions.push_back(pos_obj_vec);
  }
}

//obtain and handle the orientations values from the XML extracted data
void XsensData::orientations_extract(XsensXml xsensXml){
  vector<vector<double>> orientationValues;
  vector<string> orientationStr;

  //Get the vector of strings containing all the orientation values in each frame
  xsensXml.get_orientation(orientationStr);
  //Parse and Extract the values from each string containing all the orientation values in each frame to a bidimensional vector of double
  //In this new vector the index represent each frame of the movement
  xsensXml.data_parsing(orientationStr, orientationValues);
  orientation_quaternion or_obj;
  vector<orientation_quaternion> or_obj_vec;

  for(int frame=0; frame<this->frames; frame++){
  or_obj_vec.clear();
    for (int n_segment=0; n_segment<(orientationValues.at(frame).size())/4; n_segment++){  //each segment orientation is represented in quartenion (q0,q1,q2 and q3). The string of each frame contains the orientation representation of each segment.
      or_obj.frame = frame;
      or_obj.q0 = orientationValues[frame][4*n_segment];
      or_obj.q1 = orientationValues[frame][4*n_segment+1];
      or_obj.q2 = orientationValues[frame][4*n_segment+2];
      or_obj.q3 = orientationValues[frame][4*n_segment+3];
      or_obj.segment = segments_names.at(n_segment);
      or_obj_vec.push_back(or_obj);
    }
    orientations_quat.push_back(or_obj_vec);
  }
}


void XsensData::convert_orientations_quat2deg(){

  vector<orientation_deg> orientation_deg_vec;
  vector<orientation_quaternion> orientation_quat_vec;
  orientation_deg orientation_deg_obj;
  Orientation_quat orientation_quat_obj;

  Quaterniond q;
  Vector3d e;

  this->orientations_deg.clear();

  for(int frame = 0; frame < this->frames; frame++)
  {
    orientation_deg_vec.clear();
    orientation_quat_vec = orientations_quat.at(frame);
    for(int segment = 0; segment < 23; segment++)
    {
      orientation_quat_obj = orientation_quat_vec.at(segment);
      q.w() = orientation_quat_obj.q0;
      q.x() = orientation_quat_obj.q1;
      q.y() = orientation_quat_obj.q2;
      q.z() = orientation_quat_obj.q3;

      e = q.matrix().eulerAngles(0,1,2);

      orientation_deg_obj.x = e[0];
      orientation_deg_obj.y = e[1];
      orientation_deg_obj.z = e[2];
      orientation_deg_obj.frame = frame;
      orientation_deg_obj.segment = segments_names.at(segment);

      orientation_deg_vec.push_back(orientation_deg_obj);
    }

    orientations_deg.push_back(orientation_deg_vec);
  }

}


void XsensData::rotate_orientations(string segment_name, vector<float> rotation){
  float alpha = rotation.at(0); //rotation in x
  float beta = rotation.at(1); //rotation in y
  float gama = rotation.at(2); // rotation in z

  orientation_deg original_orientation;
  orientation_deg desired_orientation;

  for(int frame = 0; frame < this->frames; frame++){
    for(int segment = 0; segment < 23; segment++){
      if(this->segments_names.at(segment) == segment_name)
      {
        original_orientation = orientations_deg.at(frame).at(segment);
        float x,y,z;
        x = original_orientation.x;
        y = original_orientation.y;
        z = original_orientation.z;
        Matrix3d matrixX, matrixY, matrixZ, matrixRot;

        matrixX << 1, 0, 0,
                   0, cos(x), -sin(x),
                   0, sin(x), cos(x);

        matrixY << cos(y), 0, sin(y),
                   0, 1, 0,
                   -sin(y), 0, cos(y);

        matrixZ << cos(z), -sin(z), 0,
                   sin(z), cos(z), 0,
                   0, 0, 1;

        matrixRot = matrixX * matrixY * matrixZ; //rotation matrix related to the original orientation

        Matrix3d matrixAlpha, matrixBeta, matrixGama;

        matrixAlpha << 1, 0, 0,
                   0, cos(alpha), -sin(alpha),
                   0, sin(alpha), cos(alpha);

        matrixBeta << cos(beta), 0, sin(beta),
                   0, 1, 0,
                   -sin(beta), 0, cos(beta);

        matrixGama << cos(gama), -sin(gama), 0,
                   sin(gama), cos(gama), 0,
                   0, 0, 1;

        matrixRot =  matrixAlpha * matrixBeta * matrixGama * matrixRot;

        float roll = 0, pitch = 0, yaw = 0;

        roll = atan2(-matrixRot(0,1),matrixRot(0,0));
        pitch = atan2(matrixRot(0,2), sqrt(powf(matrixRot(0,0),2) + powf(matrixRot(0,1),2)));
        yaw = atan2(-matrixRot(1,2), matrixRot(2,2));

        orientations_deg.at(frame).at(segment).x = yaw;
        orientations_deg.at(frame).at(segment).y = pitch;
        orientations_deg.at(frame).at(segment).z = roll;
      }
    }
  }
}




int XsensData::get_frames_number(){
  return this->frames;
}

void XsensData::get_segments_name(vector<string> &segments_names){

  segments_names.clear();
  for(int i = 0; i < 23; i++)
    segments_names.push_back(this->segments_names.at(i));
}

int XsensData::get_jointangleZXY(int frame, string joint_name, joint_angle_zxy &joint_obj){

  for(int i=0; i<joints_angles_zxy.at(frame).size(); i++){
    if(joints_angles_zxy[frame][i].joint == joint_name){
      joint_obj = joints_angles_zxy[frame][i];
      return 1;
    }
  }
  return -1;
}

int XsensData::get_ergonomicJointangleZXY(int frame, string joint_name, ergonomicJoint_angle_zxy &ergonomicJoint_obj){

  for(int i=0; i<ergonomicJoints_angles_zxy.at(frame).size(); i++){
    if(ergonomicJoints_angles_zxy[frame][i].ergonomicJoint == joint_name){
      ergonomicJoint_obj = ergonomicJoints_angles_zxy[frame][i];
      return 1;
    }
  }
  return -1;
}

//to get the information about the position of a specific segment in a specific frame recorded
int XsensData::get_segment_position(int frame, string segment_name, position &pos_obj){

  for(int i=0; i<positions.at(frame).size(); i++){
    if(positions[frame][i].segment == segment_name){
      pos_obj = positions[frame][i];
      return 1;
    }
  }
  return -1;
}

int XsensData::get_frame_positions(int frame, vector<position> &pos_obj){

  pos_obj = positions.at(frame);
}


//to get the information about the orientation of a specific segment in a specific frame recorded
int XsensData::get_segment_orientation_quaternion(int frame, string segment_name, orientation_quaternion &or_obj){

  for(int i=0; i<orientations_quat.at(frame).size(); i++){
    if(orientations_quat[frame][i].segment == segment_name){
      or_obj = orientations_quat[frame][i];
      return 1;
    }
  }
  return -1;
}



int XsensData::get_segment_orientation_deg(int frame, string segment_name, orientation_deg &or_obj){

  for(int i=0; i<orientations_deg.at(frame).size(); i++){
    if(orientations_deg[frame][i].segment == segment_name){
      or_obj = orientations_deg[frame][i];
      return 1;
    }
  }
  return -1;
}



void XsensData::rula_evaluation(string arm_side){

  this->rula_buffer.clear();
  this->rula_scores.clear();
  Rula rula;

  //Rula variables
  upper_arm_data upper_arm = {0,0,0,0,0};
  lower_arm_data lower_arm= {0,0,0};
  wrist_data wrist = {0,0,0,0};
  neck_data neck = {0,0,0,0};
  trunk_data trunk = {0,0,0,0,0,0};
  legs_data legs = {0,0};

  joint_angle_zxy joint;
  ergonomicJoint_angle_zxy ergonomicJoint;

  //Upper Arm init

  for(int frame = 0; frame < frames; frame ++){
    //Neck rula variables
    get_jointangleZXY(frame, "jC1Head", joint);
    if(abs(joint.z) > 10) neck.bent = 1;
    else neck.bent = 0;
    if(abs(joint.x) > 10) neck.twisted = 1;
    else neck.twisted = 0;
    neck.flexion = joint.y;

    //Forearm rula variables
    if(arm_side == "right") get_jointangleZXY(frame, "jRightElbow", joint);
    else if(arm_side == "left") get_jointangleZXY(frame, "jLeftElbow", joint);

    if(abs(joint.z) > 10) lower_arm.body_midline_exceeded = 1;
    else lower_arm.body_midline_exceeded = 0;
    lower_arm.flexion = joint.y;

    //Wrist rula variables
    if(arm_side == "right") get_jointangleZXY(frame,"jRightWrist",joint);
    else if(arm_side == "left") get_jointangleZXY(frame,"jLeftWrist",joint);

    if(abs(joint.z) > 10) wrist.deviated = 1;
    else wrist.deviated = 0;
    if(abs(joint.x) > 95) wrist.twisted = 2;
    else wrist.twisted = 1;
    wrist.flexion = joint.y;

    //Back rula variables
    get_ergonomicJointangleZXY(frame,"Vertical_T8",ergonomicJoint);
    trunk.standing = TRUE;
    if(abs(ergonomicJoint.z) > 10) trunk.bending = 1;
    else trunk.bending = 0;
    if(abs(ergonomicJoint.x) > 10) trunk.twisted = 1;
    else trunk.twisted = 0;
    trunk.flexion = ergonomicJoint.y;

    //Upper Arm rula variables
    if(arm_side == "right") get_ergonomicJointangleZXY(frame,"T8_RightUpperArm",ergonomicJoint);
    else if(arm_side == "left") get_ergonomicJointangleZXY(frame,"T8_LeftUpperArm",ergonomicJoint);

    if(abs(ergonomicJoint.z) > 10) upper_arm.abducted = 1; //lateral bending
    else upper_arm.abducted = 0;
    upper_arm.flexion = ergonomicJoint.y;

    //Legs rula variables
    legs.stable = 1;

    rula.set_upper_arm(upper_arm,arm_side);
    rula.set_lower_arm(lower_arm,arm_side);
    rula.set_wrist(wrist,arm_side);
    rula.set_neck(neck);
    rula.set_trunk(trunk);
    rula.set_legs(legs);
    rula.set_load(0);

    rula.execute_rula_evalutation(arm_side);

    this->rula_scores.push_back(rula.get_total_score());
    this->rula_buffer.push_back(rula);
  }
}


void XsensData::publish_frame_positions(ros_communication ros_handler, int frame){

  vector<vector<float>> positions_buff;
  vector<float> position_vec = {0,0,0};
  vector<position> pos_obj_vec;
  int n_segments = 23;

  positions_buff.clear();
  pos_obj_vec.clear();


  pos_obj_vec = positions.at(frame);  //pass all the positios objects of a frame to the buffer of positions

  for(int segment = 0; segment < n_segments; segment ++){
    position_vec.at(0) = pos_obj_vec.at(segment).x;
    position_vec.at(1) = pos_obj_vec.at(segment).y;
    position_vec.at(2) = pos_obj_vec.at(segment).z;
    positions_buff.push_back(position_vec);
  }

  ros_handler.ros_publish_positions(positions_buff);
}


void XsensData::publish_frame_orientationsDeg(ros_communication ros_handler, int frame){

  vector<vector<float>> orientations_buff;
  vector<float> orientation_vec = {0,0,0};
  vector<orientation_deg> ori_obj_vec;
  int n_segments = 15;

  orientations_buff.clear();
  ori_obj_vec.clear();


  ori_obj_vec = orientations_deg.at(frame);  //pass all the orientations objects of a frame to the buffer of orientations

  for(int segment = 0; segment < n_segments; segment ++){
    orientation_vec.at(0) = ori_obj_vec.at(segment).x;
    orientation_vec.at(1) = ori_obj_vec.at(segment).y;
    orientation_vec.at(2) = ori_obj_vec.at(segment).z;
    orientations_buff.push_back(orientation_vec);
  }

  ros_handler.ros_publish_orientations(orientations_buff);
}



void XsensData::publish_frame_rula_status(ros_communication ros_handler, int frame,string side){

  vector<int> segment_status = {0,0,0,0,0,0,0,0};

  segment_status.at(0) = rula_buffer.at(frame).get_trunk_score_status();
  segment_status.at(1) = rula_buffer.at(frame).get_neck_score_status();
  if(side=="right"){
    segment_status.at(2) = rula_buffer.at(frame).get_upper_arm_score_status("right");
    segment_status.at(3) = rula_buffer.at(frame).get_lower_arm_score_status("right");
    segment_status.at(4) = rula_buffer.at(frame).get_wrist_score_status("right");
  }
  else if(side=="left"){
    segment_status.at(5) = rula_buffer.at(frame).get_upper_arm_score_status("left");
    segment_status.at(6) = rula_buffer.at(frame).get_lower_arm_score_status("left");
    segment_status.at(7) = rula_buffer.at(frame).get_wrist_score_status("left");
  }
  ros_handler.ros_publish_status(segment_status);
}


void XsensData::publish_data(ros_communication ros_handler, string side){

  for(int frame = 0; frame<this->frames; frame++){
    this->publish_frame_positions(ros_handler,frame);
    this->publish_frame_orientationsDeg(ros_handler,frame);
    this->publish_frame_rula_status(ros_handler,frame,side);
  }
}


void XsensData::sort_rula_evaluation(vector<Rula> &sorted_rula){
  vector<int> sorted_rula_scores;
  sorted_rula_scores.clear();

  for(int score = 1; score <= 8; score++){
    for(int frame = 0; frame < this->frames; frame++){
       if(this->rula_scores.at(frame) == score){
         sorted_rula.push_back(this->rula_buffer.at(frame));
         sorted_rula_scores.push_back(this->rula_scores.at(frame));
       }
    }
  }
}


void XsensData::write_data_files(string filename, string arm_side){

  ofstream file(filename);
  position pos;
  orientation_deg ori;
  Rula rula_obj;
  string segment_name = arm_side + "Hand";
  segment_name[0] = toupper(segment_name[0]);

  if(!file.is_open()) return;

  QDateTime curr_datetime = QDateTime::currentDateTime();
  QTime curr_time = curr_datetime.time();
  QDate curr_date = curr_datetime.date();
  file << "time: " << curr_date.toString("dd/MM/yyyy").toStdString() << " | " << curr_time.toString("hh:mm:ss").toStdString() << endl;
  file << "number of frames: " << this->get_frames_number();
  file << endl;
  for (int frame = 0; frame < this->frames; frame++){

    this->get_segment_position(frame,segment_name,pos);
    //pos.z = pos.z - 0.2758;
    this->get_segment_orientation_deg(frame,segment_name,ori);

    rula_obj = rula_buffer.at(frame);

    file << "frame: " << frame;
    file << endl;
    file << "Rula Total Score: " << rula_obj.get_total_score();
    file << endl;
    file << "Rula Upper Arm Score: " << rula_obj.get_upper_arm_score(arm_side);
    file << endl;
    file << "Rula Lower Arm Score: " << rula_obj.get_lower_arm_score(arm_side);
    file << endl;
    file << "Rula Wrist Score: " << rula_obj.get_wrist_score(arm_side);
    file << endl;
    file << "Rula Neck Score: " << rula_obj.get_neck_score();
    file << endl;
    file << "Rula Trunk Score: " << rula_obj.get_trunk_score();
    file << endl;
    file << "Rula Legs Score: " << rula_obj.get_legs_score();
    file << endl;
    file << "Human Hand Position (XYZ): " << to_string(pos.x) << " " << to_string(pos.y) << " " << to_string(pos.z);
    file << endl;
    file << "Human Hand Orientation (XYZ): " << to_string(ori.x) << " " << to_string(ori.y) << " " << to_string(ori.z);
    file << endl;
  }

}
