#include "main.h"
#include "pros/motors.h"

pros::Motor FL(10, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor FR(11, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor ML_intake(1, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MR_intake(2, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor BL(14, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor BR(18, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Fly(4, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Indexer(3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Motor Intake(8, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);

//controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
//sensor
pros::IMU inertial(5);

pros::ADIEncoder backEncoder('G', 'H', false);
pros::Rotation leftEncoder(18);
pros::Rotation rightEncoder(13);


//pistons
// pros::ADIDigitalOut shooter('A');
// pros::ADIDigitalOut pistonB1('A');
// pros::ADIDigitalOut pistonB2('A');
