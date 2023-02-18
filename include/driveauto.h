#include "main.h"
#include "cmath"
#include "globals.h"
#include "odom.h"
#include "variables.h"

#ifndef DRIVEAUTO_H
#define DRIVEAUTO_H

using namespace pros;

void shoot(int num_disks, int rpmSpeed, int timeout, int threshold, int waitMsec);
void index(int disk);

void pivot(double angle);
void turn(double angle);
void setPIDvalues();

void forwardPD(int inches, double limit, double f_kP_Theta);

void oldDriveArcPD(int leftTicks, int rightTicks, double limit, int dir);

void negative(double angle, float p, float d);
void rotate(double angle);
#endif