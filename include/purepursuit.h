#include "main.h"
#include "waypoint.h"

#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

using namespace pros;

void pathFollowPurePursuit(std::vector<Waypoint> pathToFollow);
std::vector<Waypoint> smooth(std::vector<Waypoint> roughPath, float a, float b, float tolerance);
void pathFollowNormal(std::vector<Waypoint> pathToFollow);

#endif