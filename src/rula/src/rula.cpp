#include "../include/rula/rula.h"


int tableA[18][8] = { 1,2,2,2,2,3,3,3,
                     2,2,2,2,3,3,3,3,
                     2,3,3,3,3,3,4,4,

                     2,3,3,3,3,4,4,4,
                     3,3,3,3,3,4,4,4,
                     3,4,4,4,4,4,5,5,

                     3,3,4,4,4,4,5,5,
                     3,4,4,4,4,4,5,5,
                     4,4,4,4,4,5,5,5,

                     4,4,4,4,4,5,5,5,
                     4,4,4,4,4,5,5,5,
                     4,4,4,5,5,5,6,6,

                     5,5,5,5,5,6,6,7,
                     5,6,6,6,6,7,7,7,
                     6,6,6,7,7,7,7,8,

                     7,7,7,7,7,8,8,9,
                     8,8,8,8,8,9,9,9,
                     9,9,9,9,9,9,9,9 };


int tableB[6][12] = { 1,3,2,3,3,4,5,5,6,6,7,7,
                     2,3,2,3,4,5,5,5,6,7,7,7,
                     3,3,3,4,4,5,5,6,6,7,7,7,
                     5,5,5,6,6,7,7,7,7,7,8,8,
                     7,7,7,7,7,8,8,8,8,8,8,8,
                     8,8,8,8,8,8,8,9,9,9,9,9 };

int tableC[8][7] = { 1,2,3,3,4,5,5,
                    2,2,3,4,4,5,5,
                    3,3,3,4,4,5,6,
                    3,3,3,4,5,6,6,
                    4,4,4,5,6,7,7,
                    4,4,5,6,6,7,7,
                    5,5,6,6,7,7,7,
                    5,5,6,7,7,7,7 };


int upper_lower_arm_index[6][3] = { 0,1,2,
                                    3,4,5,
                                    6,7,8,
                                    9,10,11,
                                    12,13,14,
                                    15,16,17 };

int wrist_twist_index[4][2] = { 0,1,
                                2,3,
                                4,5,
                                6,7 };

int trunk_legs_index[6][2] = { 0,1,
                               2,3,
                               4,5,
                               6,7,
                               8,9,
                               10,11};

vector<int> upper_arm_evaluation_status = {GOOD, GOOD, MODERATED, MODERATED, BAD, BAD};
vector<int> lower_arm_evaluation_status = {GOOD, MODERATED, BAD};
vector<int> wrist_evaluation_status = {GOOD, GOOD, MODERATED, BAD};
vector<int> neck_evaluation_status = {GOOD, GOOD, MODERATED, MODERATED, BAD, BAD};
vector<int> trunk_evaluation_status = {GOOD, GOOD, MODERATED, MODERATED, BAD, BAD};


Rula::Rula()
{
}

Rula::Rula(upper_arm_data upper_arm, lower_arm_data lower_arm, wrist_data wrist, neck_data neck,
           trunk_data trunk, legs_data legs){

  this->right_upper_arm = upper_arm;
  this->right_lower_arm = lower_arm;
  this->right_wrist = wrist;
  this->neck = neck;
  this->trunk = trunk;
  this->legs = legs;
  this->load_score = 0;
}


void Rula::set_upper_arm(upper_arm_data upper_arm, string side){

 if(side == "right") this->right_upper_arm = upper_arm;
 else if(side == "left") this->left_upper_arm = upper_arm;
}

void Rula::set_lower_arm(lower_arm_data lower_arm, string side){

  if(side == "right") this->right_lower_arm = lower_arm;
  else if(side == "left") this->left_lower_arm = lower_arm;
}

void Rula::set_wrist(wrist_data wrist, string side){

  if(side == "right") this->right_wrist = wrist;
  else if(side == "left") this->left_wrist = wrist;
}

void Rula::set_neck(neck_data neck){

  this->neck = neck;
}

void Rula::set_trunk(trunk_data trunk){

  this->trunk = trunk;
}

void Rula::set_legs(legs_data legs){

  this->legs = legs;
}

void Rula::set_load(int load){
  this->load_score = load;
}


void Rula::upper_arm_evaluation(string side){

 //Right Upper Arm
 if(side == "right"){
   right_upper_arm.evaluation = 1;

   if(right_upper_arm.flexion >= -20 && right_upper_arm.flexion <= 20) right_upper_arm.evaluation = 1;
   else if((right_upper_arm.flexion > 20 && right_upper_arm.flexion <= 45) || (right_upper_arm.flexion < -20)) right_upper_arm.evaluation = 2;
   else if(right_upper_arm.flexion > 45 && right_upper_arm.flexion <= 90) right_upper_arm.evaluation = 3;
   else if(right_upper_arm.flexion > 90) right_upper_arm.evaluation = 4;

   right_upper_arm.evaluation += (right_upper_arm.raised + right_upper_arm.abducted - right_upper_arm.arm_weight_supported);
   right_upper_arm.evaluation_status = upper_arm_evaluation_status.at(right_upper_arm.evaluation - 1);
 }

 //Left Upper Arm
 else if(side == "left"){
   left_upper_arm.evaluation = 1;

   if(left_upper_arm.flexion >= -20 && left_upper_arm.flexion <= 20) left_upper_arm.evaluation = 1;
   else if((left_upper_arm.flexion > 20 && left_upper_arm.flexion <= 45) || (left_upper_arm.flexion < -20)) left_upper_arm.evaluation = 2;
   else if(left_upper_arm.flexion > 45 && left_upper_arm.flexion <= 90) left_upper_arm.evaluation = 3;
   else if(left_upper_arm.flexion > 90) left_upper_arm.evaluation = 4;

   left_upper_arm.evaluation += (left_upper_arm.raised + left_upper_arm.abducted - left_upper_arm.arm_weight_supported);
   left_upper_arm.evaluation_status = upper_arm_evaluation_status.at(left_upper_arm.evaluation - 1);
 }
}

void Rula::lower_arm_evaluation(string side){

 //Right Lower Arm
 if(side == "right"){
   right_lower_arm.evaluation = 1;

   if(right_lower_arm.flexion > 60 && right_lower_arm.flexion <= 100) right_lower_arm.evaluation = 1;
   else if(right_lower_arm.flexion < 0 || (right_lower_arm.flexion >= 0 && right_lower_arm.flexion <= 60) || (right_lower_arm.flexion > 100)) right_lower_arm.evaluation = 2;

   right_lower_arm.evaluation += right_lower_arm.body_midline_exceeded;
   right_lower_arm.evaluation_status = lower_arm_evaluation_status.at(right_lower_arm.evaluation - 1);
 }

 //Left Lower Arm
 else if(side == "left"){
   left_lower_arm.evaluation = 1;

   if(left_lower_arm.flexion > 60 && left_lower_arm.flexion <= 100) left_lower_arm.evaluation = 1;
   else if(left_lower_arm.flexion < 0 || (left_lower_arm.flexion >= 0 && left_lower_arm.flexion <= 60) || (left_lower_arm.flexion > 100)) left_lower_arm.evaluation = 2;

   left_lower_arm.evaluation += left_lower_arm.body_midline_exceeded;
   left_lower_arm.evaluation_status = lower_arm_evaluation_status.at(left_lower_arm.evaluation - 1);
 }
}

void Rula::wrist_evaluation(string side){

 //Right Wrist
 if(side == "right"){
   right_wrist.evaluation = 1;

   if(right_wrist.flexion >= -1 && right_wrist.flexion <= 1) right_wrist.evaluation = 1;
   else if(right_wrist.flexion >= -15 && right_wrist.flexion <= 15) right_wrist.evaluation = 2;
   else if(right_wrist.flexion < -15 || right_wrist.flexion > 15) right_wrist.evaluation = 3;

   right_wrist.evaluation += right_wrist.twisted + right_wrist.deviated; //midline_exceeded;
   right_wrist.evaluation_status = wrist_evaluation_status.at(right_wrist.evaluation - right_wrist.twisted - 1);
 }

 //Left Wrist
 else if(side == "left"){
   left_wrist.evaluation = 1;

   if(left_wrist.flexion >= -1 && left_wrist.flexion <= 1) left_wrist.evaluation = 1;
   else if(left_wrist.flexion >= -15 && left_wrist.flexion <= 15) left_wrist.evaluation = 2;
   else if(left_wrist.flexion < -15 || left_wrist.flexion > 15) left_wrist.evaluation = 3;

   left_wrist.evaluation += left_wrist.twisted + left_wrist.deviated; //midline_exceeded;
   left_wrist.evaluation_status = wrist_evaluation_status.at(left_wrist.evaluation - left_wrist.twisted - 1);
 }
}




void Rula::neck_evaluation(){

  neck.evaluation = 1;

  if(neck.flexion >= -5 && neck.flexion <=10) neck.evaluation = 1;
  else if(neck.flexion > 10 && neck.flexion <= 20) neck.evaluation = 2;
  else if(neck.flexion > 20) neck.evaluation = 3;
  else if(neck.flexion < -5) neck.evaluation = 4;

  neck.evaluation += neck.bent + neck.twisted;
  neck.evaluation_status = neck_evaluation_status.at(neck.evaluation - 1);
}

void Rula::trunk_evaluation(){

  trunk.evaluation = 1;

  switch(trunk.standing){

  case TRUE:
  if(trunk.flexion >= -5 && trunk.flexion <= 0.1) trunk.evaluation = 1;
  else if(trunk.flexion > 0.1 && trunk.flexion <= 20) trunk.evaluation = 2;
  else if(trunk.flexion > 20 && trunk.flexion <= 60) trunk.evaluation = 3;
  else if(trunk.flexion > 60) trunk.evaluation = 4;
    break;

  case FALSE:
    if(trunk.well_suported == TRUE) trunk.evaluation = 1;
    else trunk.evaluation = 2;
    break;
  }

  trunk.evaluation += trunk.bending + trunk.twisted;
  trunk.evaluation_status = trunk_evaluation_status.at(trunk.evaluation - 1);
}

void Rula::legs_evaluation(){

  legs.evaluation = legs.stable;
}


void Rula::groupA_evaluation(string side){

  int row_index = -1;
  int column_index = -1;

  if(side == "right"){
    row_index = upper_lower_arm_index[right_upper_arm.evaluation-1][right_lower_arm.evaluation-1];
    column_index = wrist_twist_index[(right_wrist.evaluation-right_wrist.twisted)-1][right_wrist.twisted-1];
  }
  else if(side == "left"){
    row_index = upper_lower_arm_index[left_upper_arm.evaluation-1][left_lower_arm.evaluation-1];
    column_index = wrist_twist_index[(left_wrist.evaluation-left_wrist.twisted)-1][left_wrist.twisted-1];
  }

  groupA_score = tableA[row_index][column_index];
  groupA_score += load_score;
}

void Rula::groupB_evaluation(){

  int row_index = neck.evaluation - 1;
  int column_index = trunk_legs_index[trunk.evaluation-1][legs.evaluation-1];

  groupB_score = tableB[row_index][column_index];
  groupB_score += load_score;
}

void Rula::total_evaluation(){

  int row_index = groupA_score - 1;
  int column_index = groupB_score - 1;

  if(row_index > 7) row_index = 7;
  if(column_index > 6) column_index = 6;

  total_score = tableC[row_index][column_index];

}

void Rula::execute_rula_evalutation(string side){

  this->upper_arm_evaluation(side);
  this->lower_arm_evaluation(side);
  this->wrist_evaluation(side);
  this->neck_evaluation();
  this->trunk_evaluation();
  this->legs_evaluation();
  this->groupA_evaluation(side);
  this->groupB_evaluation();
  this->total_evaluation();
}

int Rula::get_upper_arm_score(string side){

  if(side == "right") return this->right_upper_arm.evaluation;
  else if(side == "left") return this->left_upper_arm.evaluation;
}


int Rula::get_lower_arm_score(string side){

  if(side == "right") return this->right_lower_arm.evaluation;
  else if(side == "left") return this->left_lower_arm.evaluation;
}


int Rula::get_wrist_score(string side){

  if(side == "right") return this->right_wrist.evaluation;
  else if(side == "left") return this->left_wrist.evaluation;
}

int Rula::get_wrist_twist_score(string side){

  if(side == "right") return this->right_wrist.twisted;
  else if(side == "left") return this->left_wrist.twisted;
}


int Rula::get_neck_score(){

  return this->neck.evaluation;
}

int Rula::get_trunk_score(){

  return this->trunk.evaluation;
}

int Rula::get_legs_score(){

  return this->legs.evaluation;
}

int Rula::get_groupA_score(){

  return this->groupA_score;
}

int Rula::get_groupB_score(){

  return this->groupB_score;
}

int Rula::get_total_score(){

  return this->total_score;
}


int Rula::get_upper_arm_score_status(string side){

  if(side == "right") return this->right_upper_arm.evaluation_status;
  else if(side == "left") return this->left_upper_arm.evaluation_status;
}


int Rula::get_lower_arm_score_status(string side){

  if(side == "right") return this->right_lower_arm.evaluation_status;
  else if(side == "left") return this->left_lower_arm.evaluation_status;
}


int Rula::get_wrist_score_status(string side){

  if(side == "right") return this->right_wrist.evaluation_status;
  else if(side == "left") return this->left_wrist.evaluation_status;
}


int Rula::get_neck_score_status(){

  return this->neck.evaluation_status;
}

int Rula::get_trunk_score_status(){

  return this->trunk.evaluation_status;
}
