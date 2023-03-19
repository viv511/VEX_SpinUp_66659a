#include "main.h"
#include "globals.h"
#include <vector>
#include "odom.h"
#include "pros/motors.h"
#include "variables.h"
#include "driveauto.h"
#include "fly.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>

#ifndef WAYPOINT_H
#define WAYPOINT_H

using namespace pros;

class Waypoint {
    private:
    float x;
    float y;
    float theta;
    // float distAt;
    // float targetVel;
    // float curvature;

    public:
    Waypoint() {
        this->x = 0;
        this->y = 0;
        this->theta = 0;
        // distAt = 0;
        // targetVel = 0;
        // curvature = 0;
    }
    Waypoint(float xCoord, float yCoord) {
        this->x = xCoord;
        this->y = yCoord;
        this->theta = 0;
        // distAt = 0;
        // targetVel = 0;
        // curvature = 0;
    }
    Waypoint(float xCoord, float yCoord, float angleTheta) {
        this->x = xCoord;
        this->y = yCoord;
        this->theta = angleTheta;
        // distAt = 0;
        // targetVel = 0;
        // curvature = 0;
    }

    float getX() {
        return this->x;
    }

    float getY() {
        return this->y;
    }
    
    float getTheta() {
        return this->theta;
    }

    // float getDist() {
    //     return distAt;
    // }

    // float getTarVel() {
    //     return targetVel;
    // }

    // float getCurv() {
    //     return curvature;
    // }

    void setX(float newX) {
        this->x = newX;
    }

    void setY(float newY) {
        this->y = newY;
    }

    void setTheta(float newTheta) {
        this->theta = newTheta;
    }

    // void setDist(float dist) {
    //     distAt = dist;
    // }

    // void setTarVel(float tvel) {
    //     targetVel = tvel;
    // }

    // void setCurv(float curv) {
    //     curvature = curv;
    // }
};

float getLength(Waypoint P);
float distance(Waypoint A, Waypoint B);
float angle(Waypoint A, Waypoint B);
float dotProduct(Waypoint A, Waypoint B);
Waypoint normalizeVect(Waypoint P);
Waypoint scalarMult(Waypoint P, float s);
Waypoint getDirVector(Waypoint A, Waypoint B);
int sign(float num);
void debug(Waypoint p);


#endif