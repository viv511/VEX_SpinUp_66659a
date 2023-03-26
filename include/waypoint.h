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
        float dist;
        float vel;
        float curv;

    public:
        Waypoint(float xCoord, float yCoord) {
            this->x = xCoord;
            this->y = yCoord;
            this->theta = 0;
            this->dist = 0;
            this->vel = 0;
            this->curv = 0;
        }
        Waypoint(float xCoord, float yCoord, float angleTheta) {
            this->x = xCoord;
            this->y = yCoord;
            this->theta = angleTheta;
            this->dist = 0;
            this->vel = 0;
            this->curv = 0;
        }

        //Getters
        float getX() {
            return this->x;
        }

        float getY() {
            return this->y;
        }
        
        float getTheta() {
            return this->theta;
        }

        float getDist() {
            return this->dist;
        }

        float getVel() {
            return this->vel;
        }

        float getCurv() {
            return this->curv;
        }

        //Setters
        void setX(float newX) {
            this->x = newX;
        }

        void setY(float newY) {
            this->y = newY;
        }

        void setTheta(float newTheta) {
            this->theta = newTheta;
        }

        void setDist(float newDist) {
            this->dist = newDist;
        }

        void setVel(float newVel) {
            this->vel = newVel;
        }

        void setCurv(float newCurv) {
            this->curv = newCurv;
        }
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