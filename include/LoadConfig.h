#ifndef LOADCONFIG_H
#define LOADCONFIG_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <map>
using namespace std;

extern  map<int,double*> globalLandmarks;
extern  map<string,double> globalParams;

static string landmarks_path = "/home/huanyu/robot_ws/src/camera_ros/config/landmarks.conf";
static string params_path    = "/home/huanyu/robot_ws/src/camera_ros/config/params.conf";

void loadConfig();

#endif /* loadConfig_h */
