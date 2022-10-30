#include "main.h"

using namespace pros;
#ifndef GLOBALS
#define GLOBALS

extern pros::Motor FL;
extern pros::Motor FR;
extern pros::Motor ML_intake;
extern pros::Motor MR_intake;
extern pros::Motor BL;
extern pros::Motor BR;

extern pros::Motor Fly;
extern pros::Motor Indexer;

extern pros::Motor Intake;

//controller
extern pros::Controller controller;
//sensor
extern pros::Imu inertial;

extern pros::ADIEncoder backEncoder;   
extern pros::Rotation leftEncoder;
extern pros::Rotation rightEncoder;

//pistons
extern pros::ADIDigitalOut endgame1;
extern pros::ADIDigitalOut endgame2;
// extern pros::ADIDigitalOut pistonB1;
// extern pros::ADIDigitalOut pistonB2;

#endif
