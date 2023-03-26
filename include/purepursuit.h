#include "globals.h"
#include <vector>
#include "odom.h"
#include "pros/motors.h"
#include "pros/misc.h"
#include <cmath>

#include "waypoint.h"

#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

using namespace pros;

void pathFollowNormal(std::vector<Waypoint> pathToFollow);

void pathFollowPurePursuit(std::vector<Waypoint> pathToFollow, float maximumVel, float maximumA, float constantK);

std::vector<Waypoint> pathGen(std::vector<Waypoint> pathToFollow, float maxVel, float maxA, float velocityK);

std::vector<Waypoint> smooth(std::vector<Waypoint> roughPath, float a, float b, float tolerance);
int findClosestPoint(Waypoint P, std::vector<Waypoint> path);
float circleLineIntersect(Waypoint start, Waypoint end, Waypoint curPos, float lookaheadRadius);
Waypoint findLookaheadPoint(std::vector<Waypoint> pathToFollow, Waypoint curPos, Waypoint prevLookAhead, int prevLookAheadIndex, float lookaheadRadius);

#endif