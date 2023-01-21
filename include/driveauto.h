#include "main.h"
#include "cmath"
#include "globals.h"
#include "odom.h"
#include "variables.h"

#ifndef DRIVEAUTO_H
#define DRIVEAUTO_H

using namespace pros;

void shoot(int num_disks, int rpmSpeed, int timeout, bool isChain);
void index(int disk);

void driveOdomAngPD(int inches, double limit, double f_kP, double f_kD, double f_kP_Theta);
void oldDriveArcPD(int leftTicks, int rightTicks, double limit, int dir);
void turn(double angle, float p, float d);
void negative(double angle, float p, float d);
void rotate(double angle);
#endif