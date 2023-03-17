#include "main.h"
#include "waypoint.h"
#include "globals.h"
#include <vector>
#include "odom.h"
#include "pros/motors.h"
#include "variables.h"
#include "driveauto.h"
#include "fly.h"
#include "cmath"
#include <iostream>
#include <fstream>
#include <filesystem>

#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

using namespace pros;

void pathFollowPurePursuit(std::vector<Waypoint> pathToFollow);
std::vector<Waypoint> smooth(std::vector<Waypoint> roughPath, float a, float b, float tolerance);
void pathFollowNormal(std::vector<Waypoint> pathToFollow);

#endif