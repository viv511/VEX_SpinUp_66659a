#include "main.h"
#include <cmath>
#include "globals.h"
#include "variables.h"
#include "waypoint.h"

#ifndef ODOM_H
#define ODOM_H
using namespace pros;

void odometry();

void setRobotPose(Waypoint newRobotPose);
Waypoint getRobotPose();


#endif