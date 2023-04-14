#include "Landmark.h"

int Landmark::getId(){
    return this->id;
}

void Landmark::setId(int id){
    this->id = id;
}

Vec3f Landmark::getLeftTop(){
    return this->leftTop;
}

void Landmark::setLeftTop(Vec3f point){
    this->leftTop = point;
}

Vec3f Landmark::getRightTop(){
    return this->rightTop;
}

void Landmark::setRightTop(Vec3f point){
    this->rightTop = point;
}

Vec3f Landmark::getLeftBottom(){
    return this->leftBottom;
}

void Landmark::setLeftBottom(Vec3f point){
    this->leftBottom = point;
}

vector<Vec3f> Landmark::getCircles(){
    return this->circles;
}

void Landmark::setCircles(vector<Vec3f> circles){
    this->circles = circles;
}
