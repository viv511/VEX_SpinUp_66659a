#include "main.h"

using namespace pros;
#ifndef GLOBALS
#define GLOBALS

extern pros::Motor FL;
extern pros::Motor FR;
extern pros::Motor ML;
extern pros::Motor MR;
extern pros::Motor BL;
extern pros::Motor BR;

extern pros::Motor Fly;
extern pros::Motor IIR;

//motor group
extern pros::Motor_Group LeftDT;
extern pros::Motor_Group RightDT;

//controller
extern pros::Controller controller;

extern pros::IMU inertial; 

extern pros::Rotation backEncoder;
extern pros::Rotation rightEncoder;

// //pistons
extern pros::ADIDigitalOut endgame1;
extern pros::ADIDigitalOut endgame2;

#endif
