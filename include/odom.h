#include "main.h"
#include "cmath"
#include "globals.h"
#include "variables.h"

#ifndef ODOM_H
#define ODOM_H
using namespace pros;

void odometry();

void setResetPoint(float xCoord, float yCoord, float newTheta);

float distError(float xCoord, float yCoord);
float angleError(float xCoord, float yCoord, bool inRad);
float findLineSlope(float sX, float sY, float eX, float eY);
float findLineYIntercept(float slope, float pX, float pY);


#endif