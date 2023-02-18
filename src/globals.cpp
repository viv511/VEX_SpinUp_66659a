#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

pros::Motor FL(5, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor ML(16, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BL(17, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor FR(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MR(18, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BR(11, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Fly(19, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor IIR(12, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

//controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//sensor
pros::IMU inertial(13);
// pistons
pros::ADIDigitalOut endgame1('G');
// pros::ADIDigitalOut endgame2('F');

pros::ADIEncoder backEncoder('A', 'B', true);
// pros::Rotation rightEncoder(10);
