#include "Camera_ros_node.h"

VideoCapture cap(0);
LocalizeData visualData;

using namespace std;
using namespace cv;

bool openCamera()
{
    //打开摄像头
    cap.set(3, 640);
    cap.set(4, 480);
    if (!cap.isOpened())
    {
        cout << "摄像头未成功打开" << endl;
        return 0;
    }
    return 1;
}

void *getVisulData(void *data)
{
    Mat srcImage;
    while (1)
    {
        cap >> srcImage;
        if (!srcImage.empty())
        {
            visualData = getVisualLocalizeData(srcImage);
        }
    }
}

Camera_start_object::Camera_start_object()
{
    //set values and serial connection
    ros::NodeHandle nh_private("~");
    this->cam_pub = n.advertise<nav_msgs::Odometry>("/gps", 20);
    this->cam_pub_raw = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gps_raw", 20);
    //open camera device
    try
    {
        cap.set(3, 640);
        cap.set(4, 480);
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("[XUHAORAN] Unable to open camera ");
    }

    if (cap.isOpened())
    {
        ROS_INFO_STREAM("[XUHAORAN] Camera opened");
    }
    else
    {
        ROS_ERROR_STREAM("[XUHAORAN] Unable to open camera ");
    }

    try
    {
        //加载配置文件
        loadConfig();
        //取配置参数
        loadVisualParams();
        //loadUWBParams();
        for (int i = 0; i < 20; i++)
        {
            Mat srcImage;
            cap >> srcImage;
            if (!srcImage.empty())
            {
                visualData = getVisualLocalizeData(srcImage);
                x_start = 1.0 * visualData.getVisualX();
                y_start = 1.0 * visualData.getVisualY();
                ROS_INFO("startup position x,y = [%f %f]", x_start, y_start);
            }
        }
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("[XUHAORAN] Unable to open camera ");
    }
}

Camera_start_object::~Camera_start_object()
{
    //释放摄像头
    cap.release();
}

bool Camera_start_object::ReadFromCamera()
{
    bool res = false;
    Mat srcImage;
    cap >> srcImage;
    if (!srcImage.empty())
    {
        visualData = getVisualLocalizeData(srcImage);
	if(visualData.getLandmarkId() > 0)
	{
	        x = (1.0 * visualData.getVisualX() - x_start) / 100;
        	y = (1.0 * visualData.getVisualY() - y_start) / 100;
		ROS_INFO("maxX1, maxY1, maxX2, maxY2 = [%f %f %f %f %d]", x, y, visualData.getVisualX(), visualData.getVisualY(), visualData.getLandmarkId());
		res = true;
	}
	else
	{
		res = false;
		ROS_INFO("Landmark_ID Error");
	}
    }
    else
    {
        res = false;
        ROS_INFO("Camera Image Error");
    }

    return res;
}

void Camera_start_object::publisherCameraSensorRaw()
{
    geometry_msgs::PoseWithCovarianceStamped GpsSensorRaw;

    GpsSensorRaw.header.stamp = ros::Time::now();
    GpsSensorRaw.header.frame_id = "gps_pub_raw";

    GpsSensorRaw.pose.pose.position.x = 0;
    GpsSensorRaw.pose.pose.position.y = 0;
    GpsSensorRaw.pose.pose.position.z = 0;
    memcpy(&GpsSensorRaw.pose.covariance, gps_pose_covariance, sizeof(gps_pose_covariance));

    cam_pub_raw.publish(GpsSensorRaw);
}

void Camera_start_object::publisherCameraSensor()
{
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    nav_msgs::Odometry gps;

    gps.header.stamp = ros::Time::now();
    gps.header.frame_id = "gps_pub";

    gps.pose.pose.position.x = x;
    gps.pose.pose.position.y = y;
    gps.pose.pose.position.z = 0.0;
    gps.pose.pose.orientation = odom_quat;

    gps.child_frame_id = this->robot_frame_id;
    gps.twist.twist.linear.x = 0.0;
    gps.twist.twist.linear.y = 0.0;
    gps.twist.twist.linear.z = 0.0;

    memcpy(&gps.pose.covariance, gps_pose_covariance, sizeof(gps_pose_covariance));
    memcpy(&gps.twist.covariance, gps_twist_covariance, sizeof(gps_twist_covariance));

    cam_pub.publish(gps);
}

bool Camera_start_object::ReadAndWriteLoopProcess()
{
    this->last_time = ros::Time::now();
    ofstream outFile;
    outFile.open("/home/huanyu/robot_ws/src/camera_ros/output/data.csv", ios::out); // 打开模式可省略

    while (ros::ok())
    {
        this->current_time = ros::Time::now();
        this->dt = (current_time - last_time).toSec();
        this->sampleFreq = 1.0f / dt;
        //Get npu data include robot move speed and action status information
        if (true == ReadFromCamera())
        {
            //calculate uwb position
            //PublisherOdom();
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

            nav_msgs::Odometry gps;

            gps.header.stamp = ros::Time::now();
            gps.header.frame_id = "gps_pub";

            gps.pose.pose.position.x = x;
            gps.pose.pose.position.y = y;
            gps.pose.pose.position.z = 0.0;
            gps.pose.pose.orientation = odom_quat;

            gps.child_frame_id = this->robot_frame_id;
            gps.twist.twist.linear.x = 0.0;
            gps.twist.twist.linear.y = 0.0;
            gps.twist.twist.linear.z = 0.0;

            memcpy(&gps.pose.covariance, gps_pose_covariance, sizeof(gps_pose_covariance));
            memcpy(&gps.twist.covariance, gps_twist_covariance, sizeof(gps_twist_covariance));

            cam_pub.publish(gps);

            ROS_INFO("[XUHAORAN] Published Odom! ");
        }

        //ofstream outFile;
        //outFile.open("/home/huanyu/robot_ws/src/camera_ros/output/data.csv", ios::out); // 打开模式可省略
        outFile << setiosflags(ios::fixed) << setprecision(1) << x << "," << y << "," << visualData.getLandmarkId() << endl;

        this->last_time = current_time;
        ros::spinOnce();
    }
}

void Camera_start_object::PublisherOdom()
{
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    //publish the transform over tf
    if (publish_odom)
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = "odom_trans";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);
        ROS_INFO("[XUHAORAN] Published Odom! ");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_controller");
    ROS_INFO("[XUHAORAN] camera controller node start! ");
    Camera_start_object Camera_Control;
    //Camera_Control.ReadFromCamera();
    Camera_Control.ReadAndWriteLoopProcess();
    return 0;
}


