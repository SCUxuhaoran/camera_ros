#include "VisualLocalize.h"
#include "Landmark.h"
#include "LoadConfig.h"

int ERRORnumber = 11100;
std::stringstream ss;
int binaryThreshold; //二值图像处理阈值
int binaryMaxValue; //二值图像处理最大值
int targetRectMin; //目标外接矩阵最小边界
int targetRectMax; //目标外接矩阵最大边界
int rectDistanceMin; //外接矩阵间最小距离
int rectDistanceMax; //外接矩阵间最大距离
float realSideLength; //实际路标边长
int dim; //路标维度
double e; //圆心确认误差范围
float MIN_MAX_DIS;//最长边最小值限定
float STEP_MAX_DIS;//单步最大距离
int DYN_FLAG; //是否动态连续检测

LocalizeData preVisualData; //上一次正常帧的数据

double getParam(String key){
    map<string,double>::iterator iter;
    iter = globalParams.find(key);
    if(iter != globalParams.end()){
        return iter->second;
    }else{
        cout << key << "未配置" << endl;
        return 0;
    }
}

void loadVisualParams(){
    binaryThreshold = getParam("binaryThreshold");
    binaryMaxValue = getParam("binaryMaxValue");
    targetRectMin = getParam("targetRectMin");
    targetRectMax = getParam("targetRectMax");
    rectDistanceMin = getParam("rectDistanceMin");
    rectDistanceMax = getParam("rectDistanceMax");
    realSideLength = getParam("realSideLength");
    dim = getParam("dim");
    e = getParam("e");
    MIN_MAX_DIS = getParam("MIN_MAX_DIS");
    STEP_MAX_DIS = getParam("STEP_MAX_DIS");
    DYN_FLAG = getParam("DYN_FLAG");
}

Vec3f getPointAffinedPos(Vec3f& src, const Point center, double angle)
{
    angle = angle * CV_PI / 180;
    int x = src[0] - center.x;
    int y = src[1] - center.y;
    src[0] = cvRound(x * cos(angle) + y * sin(angle) + center.x);
    src[1] = cvRound(-x * sin(angle) + y * cos(angle) + center.y);
    return src;
}

LocalizeData getVisualLocalizeData(Mat srcImage){
    
    Mat grayImage,binaryImage;
    //转化灰度图
    cvtColor(srcImage, grayImage, COLOR_RGB2GRAY);
    
    //计算相机中心坐标
    Point cameraCenter(grayImage.cols / 2, grayImage.rows / 2);

    //转化二值图像
    threshold(grayImage, binaryImage, binaryThreshold, binaryMaxValue, THRESH_BINARY);
    
    //寻找轮廓
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binaryImage, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());

    //获取外接矩形集合、目标矩阵集合
    vector<Rect> boundRect(contours.size());
    vector<Rect> targetRects;
    for (int i = 0; i < contours.size(); i++){
        boundRect[i] = boundingRect(Mat(contours[i]));
        if (boundRect[i].width < targetRectMax && boundRect[i].width > targetRectMin && boundRect[i].height < targetRectMax && boundRect[i].height > targetRectMin) { //圆形外界矩阵长宽限制一致
            rectangle(srcImage, Point(boundRect[i].x,boundRect[i].y), Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height),Scalar(0,255,0),1);
            targetRects.push_back(boundRect[i]);
        }
    }
    
    //将目标矩阵分堆
    vector<vector<Rect>> rectStacks;
    while (targetRects.size() != 0) {
        vector<Rect> tempRectStacks; //以参考矩阵划分的矩阵堆
        Rect temp = targetRects[0]; //随机抽取一个矩阵为参考
        tempRectStacks.push_back(temp);
        targetRects.erase(targetRects.begin());
        for (int i = 0; i < targetRects.size(); i++) {
            double distance = sqrt(pow(targetRects[i].x - temp.x, 2) + pow(targetRects[i].y - temp.y, 2));
            if (distance > rectDistanceMin && distance < rectDistanceMax) {
                tempRectStacks.push_back(targetRects[i]);
                targetRects.erase(targetRects.begin() + i);
                i--;
            }
        }
        if (tempRectStacks.size() > 3) { //超过3点才可能是路标
            rectStacks.push_back(tempRectStacks);
        }
    }
    
    //在各堆矩阵中找出圆心
    vector<vector<Vec3f>> all_circles;
    if (rectStacks.size() > 0) {
        for (int i = 0; i < rectStacks.size(); i++) {
            vector<Vec3f> temp_circles;
            vector<Rect> tempRectStacks = rectStacks[i];
            for(int j = 0; j < tempRectStacks.size(); j++){
                float c_x = tempRectStacks[j].x + (tempRectStacks[j].width / 2);
                float c_y = tempRectStacks[j].y + (tempRectStacks[j].height / 2);
                float c_r = (tempRectStacks[j].width + tempRectStacks[j].height) / 2;
                Vec3f circle = Vec3f(c_x,c_y,c_r);
                //cv::circle(srcImage, Point(c_x, c_y), 2, Scalar(0, 0, 255), 2);
                temp_circles.push_back(circle);
            }
            all_circles.push_back(temp_circles);
        }
    }
    
    //根据各路标的圆心数据，确定最终参考路标
    vector<Landmark> landmarks;
    for(int i = 0; i < all_circles.size(); i++){
        vector<Vec3f> circles = all_circles[i];
        bool check_flag = true;
        if (!circles.empty()) {
            //最长边确定左下和右上顶点
            Vec3f LeftTop, RightTop, LeftBottom;
            int max_dis,rt_i,lb_i;
            max_dis = rt_i = lb_i = 0;
            for (int j = 0; j < circles.size(); j++) {
                for (int k = 0; k < circles.size(); k++) {
                    float distance = sqrt(pow(circles[j][0] - circles[k][0], 2) + pow(circles[j][1] - circles[k][1], 2));
                    /*if(j != k && distance < rectDistanceMin){
                        check_flag = false;
                        break;
                    }*/
                    if(distance > max_dis && distance > MIN_MAX_DIS){
                        max_dis = distance;
                        RightTop = circles[j];
                        LeftBottom = circles[k];
                        rt_i = j;
                        lb_i = k;
                    }
                }
                /*if(check_flag == false){
                    break;
                }*/
            }
            /*if(check_flag == false){
                check_flag = true;
                continue;
            }*/
            
            //cv::circle(srcImage, Point(RightTop[0],RightTop[1]), 1, Scalar(0,255,0),2);//绿色
            //cv::circle(srcImage, Point(LeftBottom[0],LeftBottom[1]), 1, Scalar(0,0,255),2);//红色
            
            //垂直确定左上顶点
            for (int j = 0; j < circles.size(); j++) {
                if(j != rt_i && j != lb_i){
                    float theta = atan2(RightTop[0] - circles[j][0], RightTop[1] - circles[j][1]) - atan2(LeftBottom[0] - circles[j][0], LeftBottom[1] - circles[j][1]);
                    theta = theta * 180.0 / CV_PI;
                    theta = (theta < 0) ? (theta + 360.0) : theta;
                    if(fabs(theta - 90) < 10 || fabs(theta - 270) < 10){
                        LeftTop = circles[j];
                        break;
                    }
                }
            }
            
            double bian1 = sqrt(pow(LeftTop[0] - LeftBottom[0], 2) + pow(LeftTop[1] - LeftBottom[1], 2));
            double bian2 = sqrt(pow(LeftTop[0] - RightTop[0], 2) + pow(LeftTop[1] - RightTop[1], 2));
            if(fabs(bian1-bian2) > 20){
                cout<<"两直角边不等"<<endl;
                //continue;
            }
            
	    //cv::circle(srcImage, Point(LeftTop[0],LeftTop[1]), 1, Scalar(255,0,0),2);
 
            //调整左上和右上顶点
            bool flag = true;
            //取两点中点为参考点
            Point cankao1(LeftTop[0], LeftTop[1]);
            Point cankao2((LeftBottom[0] + RightTop[0]) / 2, (LeftBottom[1] + RightTop[1]) / 2);
            Point lbP(LeftBottom[0], LeftBottom[1]);
            Point rtP(RightTop[0], RightTop[1]);
            //求取俩向量夹角角度
            float lb_theta = atan2(cankao2.x - cankao1.x, cankao2.y - cankao1.y) - atan2(lbP.x - cankao1.x, lbP.y - cankao1.y);
            float rt_theta = atan2(cankao2.x - cankao1.x, cankao2.y - cankao1.y) - atan2(rtP.x - cankao1.x, rtP.y - cankao1.y);
            //cout << "lb_theta: " << lb_theta * 180.0 / CV_PI << ", " << "rt_theta: " << rt_theta * 180.0 / CV_PI << endl;
	    if (lb_theta > CV_PI)
                lb_theta -= 2 * CV_PI;
            if (lb_theta < -CV_PI)
                lb_theta += 2 * CV_PI;
            if (rt_theta > CV_PI)
                rt_theta -= 2 * CV_PI;
            if (rt_theta < -CV_PI)
                rt_theta += 2 * CV_PI;
            lb_theta = lb_theta * 180.0 / CV_PI;
            rt_theta = rt_theta * 180.0 / CV_PI;
	    //cout << "lb_theta: " << lb_theta << ", " << "rt_theta: " << rt_theta << endl;
            //全部改为顺时针方向,正值表达
            lb_theta = (lb_theta < 0) ? (lb_theta + 360.0) : lb_theta;
            rt_theta = (rt_theta < 0) ? (rt_theta + 360.0) : rt_theta;
	    //cout << "lb_theta: " << lb_theta << ", " << "rt_theta: " << rt_theta << endl;
            if (lb_theta > rt_theta)
            {
                flag = false;
            }
            if (!flag)
            {
		cout << "switch right_top and left_bottom" << endl;
                Vec3f temp = RightTop;
                RightTop = LeftBottom;
                LeftBottom = temp;
            }

	    cv::circle(srcImage, Point(LeftTop[0], LeftTop[1]), 2, Scalar(255, 0, 0), 2);
            cv::circle(srcImage, Point(RightTop[0], RightTop[1]), 2, Scalar(0, 255, 0), 2);
            cv::circle(srcImage, Point(LeftBottom[0], LeftBottom[1]), 2, Scalar(0, 0, 255), 2);

            cv::putText(srcImage, std::string("lt"), cv::Point2f(LeftTop[0], LeftTop[1]) + cv::Point2f(-10, 15), cv::FONT_HERSHEY_PLAIN, 1,  Scalar(255, 255, 255));
            cv::putText(srcImage, std::string("rt"), cv::Point2f(RightTop[0], RightTop[1]) + cv::Point2f(-10, 15), cv::FONT_HERSHEY_PLAIN, 1,  Scalar(255, 255, 255));
            cv::putText(srcImage, std::string("lb"), cv::Point2f(LeftBottom[0], LeftBottom[1]) + cv::Point2f(-10, 15), cv::FONT_HERSHEY_PLAIN, 1,  Scalar(255, 255, 255));
            
            
            Landmark tempLandmark;
            tempLandmark.setLeftTop(LeftTop);
            tempLandmark.setRightTop(RightTop);
            tempLandmark.setLeftBottom(LeftBottom);
            tempLandmark.setCircles(circles);
            landmarks.push_back(tempLandmark);

        }
    }
    
    vector<LocalizeData> localizeDatas;
    if(landmarks.size() > 0){
        for(int i = 0; i < landmarks.size(); i++){
            Vec3f LeftTop1 = landmarks[i].getLeftTop();
            Vec3f LeftTop = landmarks[i].getLeftTop();
            Vec3f RightTop = landmarks[i].getRightTop();
            Vec3f LeftBottom = landmarks[i].getLeftBottom();
            
            //矫正三点(图像中三点可能不垂直)
            if (LeftTop[0] == LeftBottom[0]) {
                RightTop[1] = LeftTop[1];
                //cout << "定位点坐标矫正1" << endl;
            }
            else if (LeftTop[1] == LeftBottom[1]) {
                RightTop[0] = LeftTop[0];
                //cout << "定位点坐标矫正2" << endl;
            }
            else if (LeftTop[0] == RightTop[0]) {
                LeftBottom[1] = LeftTop[1];
                //cout << "定位点坐标矫正3" << endl;
            }
            else if (LeftTop[1] == RightTop[1]) {
                LeftBottom[0] = LeftTop[0];
                //cout << "定位点坐标矫正4" << endl;
            }
            
            /* 获取坐标 */
            //计算图像中相机距离leftTop的距离
            double imgDis = sqrt(pow((cameraCenter.x - LeftTop[0]), 2) + pow((cameraCenter.y - LeftTop[1]), 2));
            //计算相机较landmark的方向
            float theta_z = atan2(LeftTop[0] - LeftBottom[0], LeftTop[1] - LeftBottom[1]) - atan2(cameraCenter.x - LeftTop[0], cameraCenter.y - LeftTop[1]);
            if (theta_z > CV_PI)
                theta_z -= 2 * CV_PI;
            if (theta_z < -CV_PI)
                theta_z += 2 * CV_PI;
            //计算相对于landmark的坐标
            double cameraX = imgDis * sin(theta_z);
            double cameraY = imgDis * cos(theta_z);
            //计算比例因子
            double disLeft = sqrt(pow(LeftTop[0] - LeftBottom[0], 2) + pow(LeftTop[1] - LeftBottom[1], 2));
            double disTop = sqrt(pow(LeftTop[0] - RightTop[0], 2) + pow(LeftTop[1] - RightTop[1], 2));
            double factor1 = realSideLength / disTop;
            double factor2 = realSideLength / disLeft;
            //换算为实际位置
            double cameraRealX = cameraX * factor1;
            double cameraRealY = cameraY * factor2;
            
            /* 获取偏转角度 */
            Point pt0(LeftTop[0], LeftTop[1]);
            Point pt2(LeftBottom[0], LeftBottom[1]);
            //为竖直向量取点
            Point pt3(pt0.x, pt0.y - 1);
            //求取俩向量夹角角度
            float theta_p = atan2(pt3.x - pt0.x, pt3.y - pt0.y) - atan2(pt0.x - pt2.x, pt0.y - pt2.y);
            if (theta_p > CV_PI)
                theta_p -= 2 * CV_PI;
            if (theta_p < -CV_PI)
                theta_p += 2 * CV_PI;
            theta_p = theta_p * 180.0 / CV_PI;
            //全部改为顺时针方向,正值表达
            theta_p = (theta_p < 0) ? ( 360.0 + theta_p ) : (theta_p);
            
            /* 获取id */
            int size = dim * dim - 4;
            int bits[size];
            //放射变换后定位点对应的新坐标，theta旋转角度为逆时针
            vector<Vec3f> circles = landmarks[i].getCircles();
            for (int j = 0; j < circles.size(); j++) {
                circles[j] = getPointAffinedPos(circles[j], cameraCenter, theta_p);
            }
            LeftTop = getPointAffinedPos(LeftTop, cameraCenter, theta_p);
            RightTop = getPointAffinedPos(RightTop, cameraCenter, theta_p);
            LeftBottom = getPointAffinedPos(LeftBottom, cameraCenter, theta_p);
            
            //x和y轴分量计算
            double dx = (RightTop[0] - LeftTop[0]) / (dim - 1);
            double dy = (LeftBottom[1] - LeftTop[1]) / (dim - 1);
            //按位检测圆是否存在
            int seq = 0, bitNum = 0, id = 0;
            int circlesNum = circles.size() - 3;
            for (int m = 1; m <= dim; m++) {
                for (int j = 1; j <= dim; j++) {
                    if ((m == 1 && j == 1) || (m == 1 && j == dim) || (m == dim && j == 1) || (m == dim && j == dim)) {
                        continue;
                    }
                    double bitx = LeftTop[0] + (j - 1) * dx;
                    double bity = LeftTop[1] + (m - 1) * dy;
                    bits[seq] = 0;
                    for (int k = 0; k < circles.size(); k++) {
                        if (fabs(circles[k][0] - bitx) <= e && fabs(circles[k][1] - bity) <= e) {
                            bits[seq] = 1;
                            bitNum++;
                            circles.erase(circles.begin() + k);
                            break;
                        }
                    }
                    seq++;
                }
            }
            //判断二进制为1的数目和圆圈数目是否相等
            if (circlesNum != bitNum) {
                cout << "--------------ID识别异常" << endl;
                imwrite("../output/id-error.jpg", srcImage);
		//ss ERROR
		ss.clear();
		ss.str("");
		ss << "../output/id-error-" << ERRORnumber << ".jpg";
		imwrite(ss.str() ,srcImage);
		ERRORnumber++;
            }
            //根据位值计算序号
            for (int q = 0; q < dim * dim - 4; q++) {
                id += bits[q] * pow(2, q);
            }
            
            /* 保存定位数据 */
            LocalizeData data;
            theta_p = (int)(theta_p * 10) / 10;
            data.setLandmarkId(id);
            data.setVisualTheta(theta_p);
            data.setVisualX(cameraRealX);
            data.setVisualY(cameraRealY);
            localizeDatas.push_back(data);

            /*if(id!=0){
                putText(srcImage,"Landmark ID:"+to_string(id),Point(LeftTop1[0],LeftTop1[1]+80),FONT_HERSHEY_SIMPLEX,1,Scalar(0,255,0),1,30);
            }*/
            
        }
    }

    if(localizeDatas.size() == 0){
	//ss ERROR
	ss.clear();
	ss.str("");
	ss << "../output/none-" << ERRORnumber << ".jpg";
	imwrite(ss.str() ,srcImage);
	ERRORnumber++;
        cout << "--------------未检测到路标" << endl;
        preVisualData.setLandmarkId(-1);
        return preVisualData;
    }else{
        LocalizeData final_data;
        float v_x,v_y,v_theta;
        int num;
        v_x = v_y = v_theta = num = 0;
        for(int i = 0; i < localizeDatas.size(); i++){
            //获取全局坐标
            map<int,double*>::iterator iter;
            iter = globalLandmarks.find(localizeDatas[i].getLandmarkId());
            if(iter != globalLandmarks.end()){
                v_x += iter->second[0] - localizeDatas[i].getVisualX();
                v_y += iter->second[1] + localizeDatas[i].getVisualY();
                v_theta += localizeDatas[i].getVisualTheta();
                num++;
/*
                if(DYN_FLAG == 0 || sqrt(pow(preVisualData.getVisualX() - x, 2) + pow(preVisualData.getVisualY() - y, 2)) < STEP_MAX_DIS){
                    v_x += x;
                    v_y += y;
                    num++;
                }*/
            }
        }
        if(num != 0){
            final_data.setLandmarkId(num);
            final_data.setVisualTheta(v_theta/num);
            final_data.setVisualX(v_x/num);
            final_data.setVisualY(v_y/num);
            
            //保存上一帧
            preVisualData.setVisualTheta(final_data.getVisualTheta());
            preVisualData.setVisualX(final_data.getVisualX());
            preVisualData.setVisualY(final_data.getVisualY());
            return final_data;
        }
	else{
            cout << "--------------id:" << " 配置异常" <<endl;
	    //ss ERROR
	    ss.clear();
	    ss.str("");
	    ss << "../output/no-id-" << ERRORnumber << ".jpg";
	    imwrite(ss.str() ,srcImage);
	    //ERRORnumber++;
            preVisualData.setLandmarkId(0);
            return preVisualData;
        }
    }
}
