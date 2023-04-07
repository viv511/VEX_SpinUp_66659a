#include "utilities.h"

//~~ MATH ~~
constexpr float PI = 3.141592;

//~~ PID ~~
constexpr float INTEGRAL_TURN_THRESHOLD = 3; 
constexpr float INTEGRAL_DRIVE_THRESHOLD = -1;
const int PID_DELAY_TIME = 10; 

//~~ MAX VELOCITY ~~

//We have 3.25 inch wheels, 600 RPM motors, 6M Drive (for sum of forces)
//Units of distance in inches
const float WHEEL_DIAMETER = 3.25;
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * PI;

//Units of time in seconds (from minute)
const float DT_MAX_RPM = 600;
const float DT_MAX_RPS = DT_MAX_RPM / 60;

const float MAX_VEL = DT_MAX_RPS * WHEEL_CIRCUMFERENCE; //inches per second

//~~ MAX ACCELERATION ~~

//https://squiggles.readthedocs.io/en/latest/constraints.html --> says its like 1.05Nm? but says to use .5 Nm

//Torque = Radius * force, find force ==> Torque / Radius
const float DT_FORCE = 0.5 / (WHEEL_CIRCUMFERENCE * 0.5); //in Newtons

//omg F = MA fr!!
const float ROBOT_WEIGHT = 0;
const float SUM_FORCES = 6 * DT_FORCE;

const float MAX_ACCEL = SUM_FORCES / ROBOT_WEIGHT; //inches per second per second





