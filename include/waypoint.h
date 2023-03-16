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