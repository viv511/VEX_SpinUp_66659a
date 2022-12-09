#include "main.h"
#include "variables.h"
#include "globals.h"

#ifndef FLY_H
#define FLY_H
using namespace pros;

extern pros::Mutex rpmMutex;
extern double target_rpm;

void flySpeed(void);

double getFlywheelRPM();
void setFlywheelRPM(double targetRPM);

double SMA_Filter(double rawData);

#endif