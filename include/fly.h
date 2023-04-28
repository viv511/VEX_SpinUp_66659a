#include "main.h"
#include <cmath>
#include <queue>
#include "globals.h"

#ifndef FLY_H
#define FLY_H
using namespace pros;

extern pros::Mutex rpmMutex;
extern pros::Mutex rpmReady;
extern bool flywheel_ready;
extern double target_rpm;

bool getToggle();
void setToggle(bool e);

void flySpeed(void);

double getFlywheelRPM();
void setFlywheelRPM(double targetRPM);
bool getReadyState();
void setReadyState(bool flywheelState);


double SMA_Filter(double rawData);
double EMA_Filter(double smaData);

#endif