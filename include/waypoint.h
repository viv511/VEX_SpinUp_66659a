#include "main.h"
#include "odom.h"
#include "cmath"
#include "variables.h"
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

    public:
    Waypoint(float xCoord, float yCoord);
    void setWaypoint(float xCoord, float yCoord);

    float getX() {
        return x;
    }

    void setX(float newX) {
        x = newX;
    }

    float getY() {
        return y;
    }

    void setY(float newY) {
        y = newY;
    }
};

void debug(Waypoint p);
float getLength(Waypoint P);
float distance(Waypoint A, Waypoint B);
Waypoint normalizeVect(Waypoint P);
Waypoint scalarMult(Waypoint P, float s);
int sign(float num);

#endif