#ifndef DATA_FILES_H
#define DATA_FILES_H

#include <QtXml>
#include <QFile>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>

using namespace std;

class XsensXml{
private:
  int frames;
  vector<string> segments;
  vector<string> sensors;
  vector<string> joints;
  vector<string> ergonomic_joints;

  vector<string> orientations;
  vector<string> positions;
  vector<string> jointAngles;
  vector<string> ergonomicJointAngles;

public:
  XsensXml();
  int loadFile(char* filename, QDomDocument *xml_file);
  void getFileInfo(QDomDocument xml_file);
  void get_jointAngles(vector<string> &jointAngles);
  void get_ergonomicJointAngles(vector<string> &ergonomicJointAngles);
  void get_positions(vector<string> &positions);
  void get_orientation(vector<string> &orientations);
  void get_joints_names(vector<string> &joints);
  void get_ergonomicJoints_names(vector<string> &ergonomic_joints);
  void get_segments_names(vector<string> &segments);
  int get_frames();
  void data_transmitUDP();
  void data_print();
  void data_parsing(vector<string> data_str,vector<vector<double>> &data_d);
};

#endif // DATA_FILES_H
