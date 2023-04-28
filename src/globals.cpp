#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

pros::Motor FL(7, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); //5 true
pros::Motor ML(9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); //16 true
pros::Motor BL(2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); // 17 true
pros::Motor FR(1, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); // 1 false
pros::Motor MR(5, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES); //18 false
pros::Motor BR(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES); //11 false

pros::Motor Fly(10, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor IIR(19, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);

//controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//sensor
pros::IMU inertial(11); //13
pros::Rotation backEncoder(10);
pros::Rotation rightEncoder(16);


// pistons
pros::ADIDigitalOut blooper('B');
pros::ADIDigitalOut endgame1('E');
// pros::ADIDigitalOut endgame2('F');
