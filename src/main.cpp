#include <opencv2/opencv.hpp>
#include "LoadConfig.h"
#include "VisualLocalize.h"

using namespace std;
using namespace cv;


int main() {
    
    ofstream outFile;
    outFile.open("/home/huanyu/robot_ws/src/camera_ros/output/data.csv", ios::out); // 打开模式可省略
    Mat srcImage;

    VideoCapture cap(0);
    //cap.set(3,1980);
    //cap.set(4,1080);
    cap.set(3,640);
    cap.set(4,480);
    
    
    cout << cap.get(cv::CAP_PROP_AUTO_EXPOSURE) << endl;
    cout << cap.get(cv::CAP_PROP_EXPOSURE) << endl;

    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    cap.set(cv::CAP_PROP_EXPOSURE, 0.01);

    cout << cap.get(cv::CAP_PROP_AUTO_EXPOSURE) << endl;
    cout << cap.get(cv::CAP_PROP_EXPOSURE) << endl;
    

    if(!cap.isOpened()){
        cout << "摄像头未成功打开" << endl;
        return 0;
    }

    // 加载配置文件
    loadConfig();
    //return 0;

    //取配置参数
    loadVisualParams();

    for(; ;){
        clock_t startTime, endTime;
        cap >> srcImage;
        if(srcImage.empty()){
            break;
        }
	
	//cv::imwrite("/home/huanyu/robot_ws/src/camera_ros/src/expoure0.01_3.jpg", srcImage);

        startTime = clock();
        // 视觉定位数据获取
        LocalizeData visualData = getVisualLocalizeData(srcImage);
        endTime = clock();

        //转换角度
        float theta = visualData.getVisualTheta();
        theta = (theta < 0) ? -theta : (360 - theta);
        cout << "单帧处理耗时：" << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;
        cout << visualData.getLandmarkId() << "," << theta << "---[" << visualData.getVisualX() << "," << visualData.getVisualY()<< "]"  << endl;
        outFile << visualData.getVisualX()/100 << "," << visualData.getVisualY()/100 << endl;
        
        namedWindow("show",WINDOW_NORMAL);
        imshow("show",srcImage);
        waitKey(20);

        //uwb定位数据获取

        //数据融合

    }
}
