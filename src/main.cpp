#include "main.h"
#include "globals.h"
#include "pros/misc.h"
#include "pros/optical.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <algorithm> 
#include <cmath>

constexpr double PI = 3.1415926535897;
constexpr double degreesToRadians = PI/180;
constexpr double radiansToDegrees = 180/PI;

//credit to 5225a's doc
//Distance from center (inches)
constexpr double L_DIST = 2.5;
constexpr double R_DIST = 2.5;
constexpr double B_DIST = 3.5;

//Diameter of wheels (inches)
constexpr double LEFT_WHEEL_DIAMETER = 2.783;
constexpr double RIGHT_WHEEL_DIAMETER = 2.783;
constexpr double BACK_WHEEL_DIAMETER = 2.783;

//Ticks per rotation
#define TICKS_PER_ROTATION_ENCODER 360.0
#define TICKS_PER_ROTATION_ROTATIONAL 4096.0

//2*PI*R = Circumference, circumference in inches / ticks per rotation can be multiplied by any number of ticks to get inches travelled
constexpr double LEFT_SPIN_IN_RATIO = (PI * LEFT_WHEEL_DIAMETER)/TICKS_PER_ROTATION_ROTATIONAL;
constexpr double RIGHT_SPIN_IN_RATIO = (PI * RIGHT_WHEEL_DIAMETER)/TICKS_PER_ROTATION_ROTATIONAL;
constexpr double BACK_SPIN_IN_RATIO = (PI * BACK_WHEEL_DIAMETER)/TICKS_PER_ROTATION_ENCODER;

double iHeading = 0;

double leftEncoderLast;
double rightEncoderLast;
double backEncoderLast;
double curLeftEncoder;
double curRightEncoder;
double curBackEncoder;

void odometry() {
	while(true) {
		// //Change in each encoder in inches
		// double deltaL = (curLeftEncoder - leftEncoderLast) * LEFT_SPIN_IN_RATIO;
		// double deltaR = (curRightEncoder - rightEncoderLast) * RIGHT_SPIN_IN_RATIO;
		// double deltaB = (curBackEncoder - backEncoderLast) * BACK_SPIN_IN_RATIO;
	
		// //Save last values
		// leftEncoderLast = curLeftEncoder;
		// rightEncoderLast = curRightEncoder;
		// backEncoderLast = curBackEncoder;

		// double theta = iHeading + ((deltaL-deltaR) / (L_DIST + R_DIST));

		// pros::lcd::print(0, "headingInertial: %f\n", inertial.get_rotation());
		// pros::lcd::print(3, "headingEncoder: %d\n", theta);

		// pros::lcd::print(1, "B: %d\n", backEncoder.get_value());
		// pros::lcd::print(2, "L: %ld\n", leftEncoder.get_position());
		// pros::lcd::print(3, "R: %ld\n", rightEncoder.get_position());

		pros::lcd::print(1, "L: %d\n", avg_l());
		
		pros::lcd::print(3, "R: %d\n", avg_r());

		pros::delay(10);
	}
}


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {

}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	reset_encoder();
	pros::lcd::initialize();
	inertial.reset();
	inertial.set_rotation(0);
	backEncoder.reset();
	leftEncoder.set_position(0);
	rightEncoder.set_position(0);
	pros::Task odom (odometry);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 * 
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

}
/**
 * 
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while(true) {
			pros::delay(10);
			
			int a4 = 120*controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
			int a3 = 120*controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
			FL.move_voltage(a4 + a3);
			FR.move_voltage(a4 - a3);
			MR.move_voltage(a4 - a3);
			ML.move_voltage(a4 + a3);
			BL.move_voltage(a4 + a3);
			BR.move_voltage(a4 - a3);
			
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
				Fly.move_velocity(0);
			}
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
				Fly.move_voltage(-12000);
			}

			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
				shooter.set_value(false);
			}
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				shooter.set_value(true);
			}

			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
				Intake.move_voltage(9000);
			}
			else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
				Intake.move_voltage(-9000);
			}
			else if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) || (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))){
				Intake.move_voltage(0);
			}

		}
}

void pivot(double angle) {
	//theta = total turn
	//angle = target angle (0 to 360 degrees)
	//startAngle = current angle of robot -inf < degrees < inf
	double theta = 0;
	int startAngle = inertial.get_rotation();

	//get relative to (0 to 360 degrees)
	startAngle = startAngle % 360;

	//two cases
	if(startAngle > 180) {
		theta = 360 - startAngle;
		theta += angle;

		if(theta >= 180) {
			//if the turn is bigger than 180, impractical - turn the other way
			theta = 360 - theta;
			theta*=-1;
		}
	}
	else if(startAngle <= 180) {
		theta = (theta + startAngle) * -1;
		theta+=angle;

		if(theta >= 180) {
			//if the turn is bigger than 180, impractical - turn the other way
			theta = 360 - theta;
			theta*=-1;
		}
	}
	//theta is final how much to turn

	double error = 0;
	double prevError = 0;
	double integral = 0;
	double derivative = 0;
	double power = 0;
	double kP = 7.8;
	double kI = 0;
	double kD = 1.45;

	double target = inertial.get_rotation() + theta;
	
	while(fabs(target - inertial.get_rotation()) > 0.2) {
		error = target - inertial.get_rotation();
		prevError = error;
		integral = integral + error*0.015;
		derivative = error*0.015 - prevError;
		if(error == 0 || fabs(target - inertial.get_rotation()) > 0.5) {
            integral = 0;
        }
		power = (error * kP) + (integral * kI) + (derivative*kD);
		power*=0.9;

		FR.move_velocity(power);
		FL.move_velocity(power);
		MR.move_velocity(power);
		ML.move_velocity(power);
		BR.move_velocity(power);
		BL.move_velocity(power);
		pros::delay(15);
	}
	
	FR.move_voltage(0);
    FL.move_voltage(0);
    MR.move_voltage(0);
	ML.move_voltage(0);
    BR.move_voltage(0);
	BL.move_voltage(0);
	
}
void turn(double angle) {
	inertial.tare_rotation();

	double error = 0;
	double prevError = 0;
	double integral = 0;
	double derivative = 0;
	double power = 0;
	double kP = 7.8;
	double kI = 0;
	double kD = 1.45;

	double target = inertial.get_rotation() + angle;
	
	while(fabs(target - inertial.get_rotation()) > 0.2) {
		error = target - inertial.get_rotation();
		prevError = error;
		integral = integral + error*0.015;
		derivative = error*0.015 - prevError;
		if(error == 0 || fabs(target - inertial.get_rotation()) > 0.5) {
            integral = 0;
        }
		power = (error * kP) + (integral * kI) + (derivative*kD);
		power*=0.9;

		FR.move_velocity(power);
		FL.move_velocity(power);
		MR.move_velocity(power);
		ML.move_velocity(power);
		BR.move_velocity(power);
		BL.move_velocity(power);
		pros::delay(15);
	}
	FL.move_voltage(0);
    ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    MR.move_voltage(0);
    BR.move_voltage(0);
}

void drive(int ticks) {
	reset_encoder();

	double error = 0;
	double prevError = 0;
	double integral = 0;
	double derivative = 0;
	double power = 0;
	double kP = 15;
	double kI = 0;
	double kD = 0;
	// double kD = 0.15;
	double target = average_encoders() + ticks;
	
	while(fabs(target - average_encoders()) > 25) {
		error = target - average_encoders();
		prevError = error;
		integral = integral + error*0.015;
		derivative = error*0.015 - prevError;
		if(error == 0 || fabs(target - average_encoders()) > 50) {
            integral = 0;
        }
		power = (error * kP) + (integral * kI) + (derivative*kD);

		FR.move_velocity(-power);
		FL.move_velocity(power);
		MR.move_velocity(-(power));
		ML.move_velocity((power));
		BR.move_velocity(-(power));
		BL.move_velocity((power));
		pros::delay(15);
	}
	ML.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	MR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    ML.move_voltage(0);
    BL.move_voltage(0);
    MR.move_voltage(0);
	FL.move_voltage(0);
    FR.move_voltage(0);
    BR.move_voltage(0);
}

void move_encoder(double ticks, double speed) {
	reset_encoder();
	
	FR.move_velocity(-speed);
	FL.move_velocity(speed);
	MR.move_velocity(-speed);
	ML.move_velocity(speed);
	BR.move_velocity(-speed);
	BL.move_velocity(speed);

	while(fabs(average_encoders())<ticks){
			pros::delay(2);
	}

	FL.move_voltage(0);
    ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    MR.move_voltage(0);
    BR.move_voltage(0);
}
void reset_encoder() {
	FL.tare_position();
	ML.tare_position();
	BL.tare_position();
	FR.tare_position();
	MR.tare_position();
	BR.tare_position();
}
double average_encoders() {
	return (fabs(FR.get_position())+fabs(FL.get_position())+fabs(BL.get_position()) +
		fabs(ML.get_position()) +
		fabs(BR.get_position()) +
		fabs(MR.get_position())) / 6;
}

double avg_r() {
	// return (fabs(FR.get_position()) + fabs(BR.get_position()) + fabs(MR.get_position())) / 3;
	return (fabs(FR.get_position()) + fabs(BR.get_position())) / 2;
}

double avg_l() {
	return (fabs(FL.get_position()) + fabs(BL.get_position())) / 2;
}

void goToPoint(double startX, double startY, double endX, double endY, double speed) {
	int changeX = 0 - startX;
	int changeY = 0 - startY;
	startX+=changeX;
	startY+=changeY;
	endX+=changeX;
	endY+=changeY;

	//in radians
	double turn = atan2(endY, endX) * radiansToDegrees;
	pivot(turn);
}
