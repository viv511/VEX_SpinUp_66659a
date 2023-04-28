#include "main.h"
#include <cmath>
#include "globals.h"
#include "waypoint.h"

#ifndef ODOM_H
#define ODOM_H
using namespace pros;

void odometry();

void resetX();
void moveTo(Waypoint P);
void setRobotPose(Waypoint newRobotPose);
Waypoint getRobotPose();
bool robotSettled(Waypoint A);

Waypoint scalarMult(Waypoint P, float s);
float distance(Waypoint A, Waypoint B);
float angle(Waypoint A, Waypoint B);
Waypoint normalizeVect(Waypoint P);
Waypoint getDirVector(Waypoint A, Waypoint B);
float dotProduct(Waypoint A, Waypoint B);
float getLength(Waypoint P);
int numbersign(float num);
void debug(Waypoint p);


#endif