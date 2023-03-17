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
    float distAt;
    float targetVel;
    float curvature;

    public:
    Waypoint() {
        x = 0;
        y = 0;
        distAt = 0;
        targetVel = 0;
        curvature = 0;
    }
    Waypoint(float xCoord, float yCoord) {
        x = xCoord;
        y = yCoord;
        distAt = 0;
        targetVel = 0;
        curvature = 0;
    }

    float getX() {
        return x;
    }

    float getY() {
        return y;
    }

    float getDist() {
        return distAt;
    }

    float getTarVel() {
        return targetVel;
    }

    float getCurv() {
        return curvature;
    }

    void setX(float newX) {
        x = newX;
    }

    void setY(float newY) {
        y = newY;
    }

    void setDist(float dist) {
        distAt = dist;
    }

    void setTarVel(float tvel) {
        targetVel = tvel;
    }

    void setCurv(float curv) {
        curvature = curv;
    }
};

void debug(Waypoint p);
float getLength(Waypoint P);
float distance(Waypoint A, Waypoint B);
Waypoint normalizeVect(Waypoint P);
Waypoint scalarMult(Waypoint P, float s);
int sign(float num);

#endif