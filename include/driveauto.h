#include "main.h"
#include "cmath"
#include "globals.h"
#include "odom.h"
#include "variables.h"

#ifndef DRIVEAUTO_H
#define DRIVEAUTO_H

using namespace pros;

void shoot(int num_disks, int rpmSpeed, int timeout, int threshold, int waitMsec);
void bucket(int rpmSpeed, int threshold, int timeout, int wait);
void index(int disk);

void pivot(double angle);
void turn(float angle);
void setPIDvalues();

void forwardPD(float inches, double limit);

void oldDriveArcPD(int leftTicks, int rightTicks, double limit, int dir);

void negative(double angle, float p, float d);
void rotate(double angle);
#endif