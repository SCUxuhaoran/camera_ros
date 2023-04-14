#include "LocalizeData.h"

int LocalizeData::getLandmarkId(){
    return this->landmark_id;
}

void LocalizeData::setLandmarkId(int id){
    this->landmark_id = id;
}

double LocalizeData::getVisualTheta(){
    return this->visual_theta;
}
void LocalizeData::setVisualTheta(double visual_theta){
    this->visual_theta = visual_theta;
}

float LocalizeData::getVisualX(){
    return this->visual_x;
}

void LocalizeData::setVisualX(float visual_x){
    this->visual_x = visual_x;
}

float LocalizeData::getVisualY(){
    return this->visual_y;
}

void LocalizeData::setVisualY(float visual_y){
     this->visual_y = visual_y;
}

float LocalizeData::getUwbX(){
    return this->uwb_x;
}

void LocalizeData::setUwbX(float uwb_x){
     this->uwb_x = uwb_x;
}

float LocalizeData::getUwbY(){
    return this->uwb_y;
}

void LocalizeData::setUwbY(float uwb_y){
    this->uwb_y = uwb_y;
}

float LocalizeData::getFinalX(){
    return this->final_x;
}

void LocalizeData::setFinalX(float final_x){
    this->final_x = final_x;
}

float LocalizeData::getFinalY(){
     return this->final_y;
}

void LocalizeData::setFinalY(float final_y){
    this->final_y = final_y;
}

float LocalizeData::getDTheta(){
    return this->d_theta;
}
void LocalizeData::setDTheta(float d_theta){
    this->d_theta = d_theta;
}
