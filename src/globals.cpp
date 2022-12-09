#include "main.h"
#include "pros/motors.h"

pros::Motor FL(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor ML(9, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BL(4, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor FR(10, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MR(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BR(6, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Fly(20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Roller(18, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor IIR(5, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor_Group LeftDT ({FL, ML, BL});
pros::Motor_Group RightDT ({FR, MR, BR});

//controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//sensor
pros::IMU inertial(3);
// pistons
// pros::ADIDigitalOut endgame1('G');
// pros::ADIDigitalOut endgame2('F');

pros::ADIEncoder backEncoder('G', 'H', false);
pros::Rotation rightEncoder(1);
