#ifndef VISUALLOCALIZE_H
#define VISUALLOCALIZE_H

#include <opencv2/opencv.hpp>
#include "visualLocalize.h"
#include "LocalizeData.h"

using namespace std;
using namespace cv;

void loadVisualParams();
double getParam(String key);
LocalizeData getVisualLocalizeData(Mat srcImage);
Vec3f getPointAffinedPos(Vec3f& src, const Point center, double angle);


#endif /* visualLocalize_h */
