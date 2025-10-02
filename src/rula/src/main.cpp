#include "../include/rula/data_movement.h"
#include "../include/rula/data_files.h"
#include "../include/rula/rula.h"
#include "../include/rula/ros_communication.h"
#include <QApplication>
#include <termios.h>
#include <unistd.h>

#define SERVER_PORT 9763
#define BUFF_LEN 1024

#define PI 3.141592

char getch() {
    char buf = 0;
    struct termios old = {0};
    tcgetattr(0, &old);
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    tcsetattr(0, TCSANOW, &old);
    read(0, &buf, 1);
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    tcsetattr(0, TCSADRAIN, &old);
    return buf;
}


int main(int argc, char *argv[]){

  //XML HANDLE
  QDomDocument xml_file;

  XsensXml xsensXml;
  XsensData xsensData;
  Rula rula;
  vector<Rula> sorted_rula;
  sorted_rula.clear();

  ros::init(argc, argv, "XsensROSComunication");
  ros::NodeHandle n;
  ros_communication ros_handler(argc,argv,n);

  char* filename = argv[1];
  if(xsensXml.loadFile(filename, &xml_file) == 0){exit;}

  xsensXml.getFileInfo(xml_file);
  xsensData.frames_extract(xsensXml);

  xsensData.segments_names_extract(xsensXml);
  xsensData.joints_names_extract(xsensXml);
  xsensData.ergonomicJoints_names_extract(xsensXml);

  xsensData.jointAnglesZXY_extract(xsensXml);
  xsensData.ergonomicJointAnglesZXY_extract(xsensXml);
  xsensData.positions_extract(xsensXml);
  xsensData.orientations_extract(xsensXml);
  xsensData.convert_orientations_quat2deg();

  string arm_side = "right";
  xsensData.rula_evaluation(arm_side);

  QString file_name = filename;
  file_name.replace(".xml",".txt");
  file_name.replace("xml_files","txt_files");
  xsensData.write_data_files(file_name.toStdString(),arm_side);
  printf("%d analyzed frames from %s \n",xsensData.get_frames_number(), filename);

  sleep(2);
  xsensData.publish_data(ros_handler,arm_side);

 n.shutdown();
 return 0;

  /*sleep(2);
  vector<position> pos;
  xsensData.publish_frame_positions(ros_handler,frame);
  xsensData.publish_frame_orientationsDeg(ros_handler,frame);
  xsensData.publish_frame_rula_status(ros_handler,frame,arm_side);*/

}
