#ifndef LANDMARK_H
#define LANDMARK_H

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

class Landmark{
private:
    int id;
    Vec3f leftTop;
    Vec3f rightTop;
    Vec3f leftBottom;
    vector<Vec3f> circles;
public:
    int getId();
    void setId(int id);
    Vec3f getLeftTop();
    void setLeftTop(Vec3f point);
    Vec3f getRightTop();
    void setRightTop(Vec3f point);
    Vec3f getLeftBottom();
    void setLeftBottom(Vec3f point);
    vector<Vec3f> getCircles();
    void setCircles(vector<Vec3f> circles);
};

#endif /* Landmark_h */
