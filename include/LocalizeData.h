#ifndef LOCALIZEDATA_H
#define LOCALIZEDATA_H

class LocalizeData{
private:
    int landmark_id;
    double visual_theta;
    float visual_x;
    float visual_y;
    float uwb_x;
    float uwb_y;
    float final_x;
    float final_y;
    float d_theta;

public:
    int getLandmarkId();
    void setLandmarkId(int id);
    double getVisualTheta();
    void setVisualTheta(double theta);
    float getVisualX();
    void setVisualX(float visual_x);
    float getVisualY();
    void setVisualY(float visual_y);
    float getUwbX();
    void setUwbX(float uwb_x);
    float getUwbY();
    void setUwbY(float uwb_y);
    float getFinalX();
    void setFinalX(float final_x);
    float getFinalY();
    void setFinalY(float final_y);
    float getDTheta();
    void setDTheta(float d_theta);
};

#endif /* LocalizeData_h */
