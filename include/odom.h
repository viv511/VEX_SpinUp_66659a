#include "main.h"
#include "variables.h"
#include "globals.h"

#ifndef ODOM_H
#define ODOM_H
using namespace pros;

void odometry();

void setResetPoint(float xCoord, float yCoord, float newTheta);

#endif