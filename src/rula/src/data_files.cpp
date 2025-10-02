#include "../include/rula/data_files.h"


XsensXml::XsensXml(){}

int XsensXml::loadFile(char* filename, QDomDocument *xml_file){

  QFile f(filename);

  if(!(f.open(QIODevice::ReadOnly))){
    perror("Error in loading file");
    return 0;
  }

  xml_file->setContent(&f);
  f.close();
  return 1;

}

void XsensXml::getFileInfo(QDomDocument xml_file){

  QDomElement root = xml_file.documentElement();

  QString type = root.tagName();
  QString board = root.attribute("version", "0");

  QDomElement child0Element = root.firstChild().toElement();
  QDomElement child1Element;
  QDomElement child2Element;
  QDomElement child3Element;

  while(!child0Element.isNull()){
    QString name_test = child0Element.tagName();
    if(child0Element.tagName() == "subject"){
      child1Element = child0Element.firstChild().toElement();
      while(!child1Element.isNull()){
        if(child1Element.tagName() == "segments"){
        child2Element = child1Element.firstChild().toElement();
          while(!child2Element.isNull()){
          segments.push_back(child2Element.attribute("label").toStdString());
          child2Element = child2Element.nextSibling().toElement();
          }
        }
        else if(child1Element.tagName() == "sensors"){
        child2Element = child1Element.firstChild().toElement();
          while(!child2Element.isNull()){
          sensors.push_back(child2Element.attribute("label").toStdString());
          child2Element = child2Element.nextSibling().toElement();
          }
        }
        else if(child1Element.tagName() == "joints"){
          child2Element = child1Element.firstChild().toElement();
            while(!child2Element.isNull()){
            joints.push_back(child2Element.attribute("label").toStdString());
            child2Element = child2Element.nextSibling().toElement();
            }
        }
        else if(child1Element.tagName() == "ergonomicJointAngles"){
          child2Element = child1Element.firstChild().toElement();
            while(!child2Element.isNull()){
            ergonomic_joints.push_back(child2Element.attribute("label").toStdString());
            child2Element = child2Element.nextSibling().toElement();
            }
        }
        else if(child1Element.tagName() == "frames"){
        frames = 0;
        child2Element = child1Element.firstChild().toElement();
          while(!child2Element.isNull()){
            if(child2Element.attribute("type")=="normal"){
            frames++;
            child3Element = child2Element.firstChild().toElement();
              while(!child3Element.isNull()){
                if(child3Element.tagName() == "orientation"){orientations.push_back(child3Element.firstChild().toText().data().toStdString());}
                else if(child3Element.tagName() == "position"){positions.push_back(child3Element.firstChild().toText().data().toStdString());}
                else if(child3Element.tagName() == "jointAngle"){jointAngles.push_back(child3Element.firstChild().toText().data().toStdString());}
                else if(child3Element.tagName() == "jointAngleErgo"){ergonomicJointAngles.push_back(child3Element.firstChild().toText().data().toStdString());}
              child3Element = child3Element.nextSibling().toElement();
              }
            }
            child2Element = child2Element.nextSibling().toElement();
          }
        }
        child1Element = child1Element.nextSibling().toElement();
      }
    }
    child0Element = child0Element.nextSibling().toElement();
  }
}

void XsensXml::get_jointAngles(vector<string> &jointAngles){
  jointAngles = this->jointAngles;
}

void XsensXml::get_ergonomicJointAngles(vector<string> &ergonomicJointAngles){
  ergonomicJointAngles = this->ergonomicJointAngles;
}

void XsensXml::get_positions(vector<string> &positions){
  positions = this->positions;
}

void XsensXml::get_orientation(vector<string> &orientations){
  orientations = this->orientations;
}

void XsensXml::get_joints_names(vector<string> &joints){
  joints = this->joints;
}

void XsensXml::get_ergonomicJoints_names(vector<string> &ergonomic_joints){
  ergonomic_joints = this->ergonomic_joints;
}

void XsensXml::get_segments_names(vector<string> &segments){
  segments = this->segments;
}

int XsensXml::get_frames(){
  return frames;
}

void XsensXml::data_print(){

  printf("\norientation:  %s\n", orientations.at(0).c_str());
  printf("position:  %s\n", positions.at(0).c_str());
  printf("jointAngles:  %s\n", jointAngles.at(0).c_str());
}

//General Function - Parses the string data extracted from xml file into a double 2d vector
void XsensXml::data_parsing(vector<string> data_str,vector<vector<double>> &data_d){

  size_t sz = 0;
  vector<double> data_frame;

  for(int i=0; i < data_str.size(); i++){
    data_frame.clear();
    sz = 0;
    int n_delimiters = std::count(data_str.at(i).begin(),data_str.at(i).end(), ' ');
    for(int j=0; j < (n_delimiters + 1); j++){
      data_frame.push_back((stod(data_str.at(i).substr(sz))));
      sz++;
      sz = data_str.at(i).find(' ', sz);
    }
    data_d.push_back(data_frame);
  }
}

