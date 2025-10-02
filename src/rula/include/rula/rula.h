#ifndef RULA_H
#define RULA_H

#define TRUE 1
#define FALSE -1
#define GOOD 1
#define MODERATED 2
#define BAD 3

#include <vector>
#include <string>

using namespace std;

typedef struct Upper_arm{
  float flexion;
  int raised;
  int abducted;
  int arm_weight_supported;
  int evaluation;
  int evaluation_status;
}upper_arm_data;

typedef struct Lower_arm{
  float flexion;
  int body_midline_exceeded;
  int evaluation;
  int evaluation_status;
}lower_arm_data;

typedef struct wrist{
  float flexion;
  //int midline_exceeded;
  int deviated;
  int twisted;
  int evaluation;
  int evaluation_status;
}wrist_data;

typedef struct Neck{
  float flexion;
  int twisted;
  int bent;
  int evaluation;
  int evaluation_status;
}neck_data;

typedef struct Trunk{
  bool standing;
  bool well_suported;
  float flexion;
  int twisted;
  int bending;
  int evaluation;
  int evaluation_status;
}trunk_data;

typedef struct Legs{
  int stable;
  int evaluation;
}legs_data;


class Rula
{
private:
  upper_arm_data right_upper_arm;
  lower_arm_data right_lower_arm;
  wrist_data right_wrist;
  upper_arm_data left_upper_arm;
  lower_arm_data left_lower_arm;
  wrist_data left_wrist;
  neck_data neck;
  trunk_data trunk;
  legs_data legs;
  int groupA_score;
  int groupB_score;
  int load_score;
  int total_score;

public:
  Rula();
  Rula(upper_arm_data upper_arm, lower_arm_data lower_arm, wrist_data wrist, neck_data neck, trunk_data trunk, legs_data legs);
  void set_upper_arm(upper_arm_data upper_arm, string side);
  void set_lower_arm(lower_arm_data lower_arm, string side);
  void set_wrist(wrist_data wrist, string side);
  void set_neck(neck_data neck);
  void set_trunk(trunk_data trunk);
  void set_legs(legs_data legs);
  void set_load(int load);
  void upper_arm_evaluation(string side);
  void lower_arm_evaluation(string side);
  void wrist_evaluation(string side);
  void neck_evaluation();
  void trunk_evaluation();
  void legs_evaluation();
  void groupA_evaluation(string side);
  void groupB_evaluation();
  void total_evaluation();
  void execute_rula_evalutation(string side);
  int get_upper_arm_score(string side);
  int get_lower_arm_score(string side);
  int get_wrist_score(string side);
  int get_wrist_twist_score(string side);
  int get_neck_score();
  int get_trunk_score();
  int get_legs_score();
  int get_groupA_score();
  int get_groupB_score();
  int get_total_score();

  int get_upper_arm_score_status(string side);
  int get_lower_arm_score_status(string side);
  int get_wrist_score_status(string side);
  int get_neck_score_status();
  int get_trunk_score_status();
};

#endif // RULA_H
