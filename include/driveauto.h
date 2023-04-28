#include "main.h"
#include "cmath"
#include "globals.h"
#include "odom.h"

#ifndef DRIVEAUTO_H
#define DRIVEAUTO_H

using namespace pros;

void shoot(float rpm);
void move(float dist, float limit);
void pivot(double angle);
void turn(float angle);
void swing(float angle, bool leftSwing, float otherPower);

float lim(float input, float limitPCT);
void powerMV(float l, float r);
void brake();
void setHold();
void setCoast();

#endif