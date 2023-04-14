#ifndef LOAD_CONFIG_H
#define LOAD_CONFIG_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <map>
using namespace std;

extern  map<int,double*> globalLandmarks;
extern  map<int,double*> globalAnchors;
extern  map<string,double> globalParams;

static string landmarks_path  = "/home/huanyu/robot_ws/src/camera_ros/config/landmarks.conf";
static string anchors_path    = "/home/huanyu/robot_ws/src/camera_ros/config/anchors.conf";
static string params_path     = "/home/huanyu/robot_ws/src/camera_ros/config/params.conf";

double getParam(string key);
void loadConfig();

#endif
