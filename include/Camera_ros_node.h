#ifndef CAMERA_ROS_NODE_H
#define CAMERA_ROS_NODE_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <iomanip>
#include "Landmark.h"
#include "LoadConfig.h"
#include "LocalizeData.h"
#include "VisualLocalize.h"
#include "ros/ros.h"
#include <iostream>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

#define ANCHOR_NUM 3
#define ANCHOR_DIS_START 6
#define MAX_BUFF_SIZE 16

const double gps_pose_covariance[36] = {1e-6, 0, 0, 0, 0, 0,
                                         0, 1e-6, 0, 0, 0, 0,
                                         0, 0, 1e6, 0, 0, 0,
                                         0, 0, 0, 1e6, 0, 0,
                                         0, 0, 0, 0, 1e6, 0,
                                         0, 0, 0, 0, 0, 1e6};

const double gps_pose_covariance2[36] = {1e-6, 0, 0, 0, 0, 0,
                                          0, 1e-6, 0, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e6};

const double gps_twist_covariance[36] = {1e-6, 0, 0, 0, 0, 0,
                                          0, 1e-6, 0, 0, 0, 0,
                                          0, 0, 1e6, 0, 0, 0,
                                          0, 0, 0, 1e6, 0, 0,
                                          0, 0, 0, 0, 1e6, 0,
                                          0, 0, 0, 0, 0, 1e6};

const double gps_twist_covariance2[36] = {1e-6, 0, 0, 0, 0, 0,
                                           0, 1e-6, 0, 0, 0, 0,
                                           0, 0, 1e6, 0, 0, 0,
                                           0, 0, 0, 1e6, 0, 0,
                                           0, 0, 0, 0, 1e6, 0,
                                           0, 0, 0, 0, 0, 1e6};

/*
const double gps_pose_covariance[9] = {1e6, 0, 0,
                                        0, 1e6, 0,
                                        0, 0, 1e-6};

const double gps_pose_covariance2[9] = {1e6, 0, 0,
                                         0, 1e6, 0,
                                         0, 0, 1e-6};

const double gps_twist_covariance[9] = {1e6, 0, 0,
                                         0, 1e6, 0,
                                         0, 0, 1e-6};

const double gps_twist_covariance2[9] = {1e6, 0, 0,
                                          0, 1e6, 0,
                                          0, 0, 1e-6};
*/

class Camera_start_object
{
public:
    Camera_start_object();
    ~Camera_start_object();

    /* Read/Write data from ttyUSB */
    bool ReadFromCamera();
    bool WriteToCamera(unsigned char *);
    bool ReadAndWriteLoopProcess();

    /* This node Publisher topic and tf */
    void PublisherOdom();
    void publisherCameraSensor();
    void publisherCameraSensorRaw();

    serial::Serial Camera_Serial; //声明串口对象

private:
    int baud_data;
    string usart_port, robot_frame_id;
    bool publish_odom;

    /** Ros node define*/
    ros::NodeHandle n;
    ros::Time current_time, last_time;
    double dt; //used in loopProcess caculate time difference
    //use robot_pose_ekf to calculate position, using relative position
    double x;       //current x position
    double y;       //current y position
    double x_start; //start x position
    double y_start; //start y position
    double th;      //used in PublishOdom TF tree
    double vx;
    double vy;
    double vth;
    float sampleFreq;

    ros::Subscriber cam_position_sub;
    ros::Publisher cam_position_pub, cam_pub, cam_pub_raw;

    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformListener odom_listener;
};

#endif
