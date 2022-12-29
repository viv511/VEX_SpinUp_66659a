#include "main.h"
#include "cmath"
#include "globals.h"
#include "odom.h"
#include "variables.h"

#ifndef DRIVEAUTO_H
#define DRIVEAUTO_H

using namespace pros;

void driveAngPD(int ticks, double limit);
void driveOdomAngPD(int inches, double limit, double f_kP, double f_kD, double f_kP_Theta);

#endif