#include "main.h"
#include "globals.h"
#include "odom.h"
#include "fly.h"
#include "variables.h"
#include "driveauto.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>

pros::Motor_Group LeftDT ({FL, ML, BL});
pros::Motor_Group RightDT ({FR, MR, BR});

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
	pros::lcd::initialize();

	inertial.reset(true);
	inertial.set_rotation(0);

	pros::Task odomStuff (odometry);
	pros::Task flywheelStuff (flySpeed);
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
	RollerSide();
	// nonRollerStraight();
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
float tankL = 0;
float tankR = 0;

void opcontrol() {
	LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

	while(true) {

		//TESTING
		// controller.rumble(".");

		tankL = 80 * controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		tankR = 80 * controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		LeftDT.move_voltage(tankL+tankR);
		RightDT.move_voltage(tankL-tankR);


		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
				controller.rumble(".");
			}
		}

		//Intake
		//Roller
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			IIR.move_voltage(-8000);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			IIR.move_voltage(12000);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			IIR.move_voltage(-12000);
		}
		else{
			IIR.move_voltage(0);
		}

		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			// driveOdomAngPD(48, 1, 550, 150, 0);
			// turn(90);
		}

		pros::delay(20);
		pros::lcd::print(2, "inertial: %f\n", inertial.get_rotation());
	}
}

void RollerSide() {
	flyState = true;
	setFlywheelRPM(2400);
	LeftDT.move_voltage(4000);
	RightDT.move_voltage(4000);

	IIR.move_voltage(-6000);
	pros::delay(100);
	IIR.move_voltage(0);

	driveOdomAngPD(-8, 0.5, 400, 100, 0);
	turn(-10);

	pros::delay(3000);

	controller.rumble(".");
	IIR.move_voltage(-12000);
	pros::delay(150);
	IIR.move_voltage(0);	

	pros::delay(1000);

	controller.rumble(".");
	IIR.move_voltage(-12000);
	pros::delay(200);
	IIR.move_voltage(0);

	flyState = false;
}

void nonRollerStraight() {
	flyState = true;
	setFlywheelRPM(2200);
	IIR.move_voltage(12000);
	
	driveOdomAngPD(30, 0.8, 550, 200, 0);
	IIR.move_voltage(0);
	controller.rumble(".");
	pros::delay(100);
	turn(-138);
	
	pros::delay(1500);

	controller.rumble(".");
	IIR.move_voltage(-12000);
	pros::delay(150);
	IIR.move_voltage(0);	

	pros::delay(500);

	controller.rumble(".");
	IIR.move_voltage(-12000);
	pros::delay(200);
	IIR.move_voltage(0);

	pros::delay(1000);

	controller.rumble(".");
	IIR.move_voltage(-12000);
	pros::delay(500);
	IIR.move_voltage(0);

	flyState = false;
}

void nonRoller() {
	driveOdomAngPD(25, 0.8, 550, 0, 50);
	rotate(90);
	driveOdomAngPD(6, 1, 400, 0, 0);

	IIR.move_voltage(-6000);
	pros::delay(100);
	IIR.move_voltage(0);

	driveOdomAngPD(-3, 1, 400, 0, 50);

	pros::delay(200);

	turn(125);

	IIR.move_voltage(12000);
	driveOdomAngPD(40, 0.5, 400, 0, 25);

	flyState = true;
	setFlywheelRPM(2800);
	pros::delay(100);

	turn(-97);
	
	driveOdomAngPD(-9, 0.8, 400, 0, 100);

	pros::delay(2000);
	IIR.move_voltage(0);
	controller.rumble(".");
	IIR.move_voltage(-12000);
	pros::delay(150);
	IIR.move_voltage(0);	


	pros::delay(1200);
	controller.rumble(".");
	IIR.move_voltage(-12000);
	pros::delay(200);
	IIR.move_voltage(0);

	pros::delay(1000);
	controller.rumble(".");
	IIR.move_voltage(-12000);
	pros::delay(500);
	IIR.move_voltage(0);


	flyState = false;
}

void DrivePD(double ticks, double limit) {
	//limit = decimal percent (0.00-1.00)
	reset_encoder();
	double target = ticks;
	double error = ticks;
	double latPower = 0;
	double angPower = 0;
	double prevError = error;
	double derivative = 0;
	const double kP = 18;
	const double kD = 0;

	do {
		error = target - average_encoders();
		derivative = (error - prevError);
		prevError = error;
		
		latPower = (error * kP + derivative * kD);
		if(latPower > (12000 * limit)) {
			latPower = 12000 * limit;
		}
		
		LeftDT.move_voltage(latPower);
		RightDT.move_voltage(latPower);

		pros::delay(10);

		// if(average_vel() < 10) {
		// 	break;
		// }

	} while(fabs(target - average_encoders()) > 10);

	LeftDT.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	RightDT.set_brake_modes(E_MOTOR_BRAKE_HOLD);
	LeftDT.move_voltage(0);
	RightDT.move_voltage(0);
}

void backTime(int speed, double sec) {
	LeftDT.move_velocity(-speed);
	RightDT.move_velocity(-speed);
	pros::delay(sec);
	LeftDT.move_velocity(0);
	RightDT.move_velocity(0);
}

void reset_encoder() {
	LeftDT.tare_position();
	RightDT.tare_position();
}
double average_encoders() {
	return (fabs(FR.get_position())+fabs(FL.get_position())+fabs(BL.get_position()) + fabs(BR.get_position()) + fabs(MR.get_position()) + fabs(ML.get_position()))/ 6;
}

double average_vel() {
	return (fabs(MR.get_actual_velocity())+fabs(ML.get_actual_velocity())+fabs(FR.get_actual_velocity())+fabs(FL.get_actual_velocity())+fabs(BL.get_actual_velocity()) + fabs(BR.get_actual_velocity()))/6;
}


void progSkills() {
	int variable = 1;
	// roller
	// forward turn -45 and drive forward more 
	// intake 3rd disk and get second roller
	// turn to goal and drive straight 
	// shoot 3 disks 
	// back up to original spot 
	// turn 45 degrees
	// intake 3 disks
	// turn and shoot 3 disks
	// go back to center
	// turn on disk line and pick up 2
	// go between stacks to get to roller watch movement on4082b
	// get 4th roller like the beginning with disk = 3
	// drive and shoot 3
}

// void move_encoder(double ticks, double speed) {
// 	reset_encoder();
	
// 	FR.move_velocity(speed);
// 	FL.move_velocity(speed);
// 	BR.move_velocity(speed);
// 	BL.move_velocity(speed);

// 	while(fabs(average_encoders())<ticks){
// 			pros::delay(2);
// 	}

// 	ML.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	MR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);

//     ML.move_voltage(0);
//     BL.move_voltage(0);
//     MR.move_voltage(0);
// 	FL.move_voltage(0);
//     FR.move_voltage(0);
//     BR.move_voltage(0);

// }


// void SpecialPD(int leftTicks, int rightTicks, double limit) {
// 	reset_encoder();

// 	int targetLeft = leftTicks;
// 	int targetRight = rightTicks;

// 	int leftError = targetLeft;
// 	int rightError = targetRight;

// 	int prevLeftError = leftError;
// 	int prevRightError = rightError;
// 	int derivLeft, derivRight;

// 	const float kP = 15;
// 	const float kD = 0;

// 	float leftPower, rightPower;

// 	do{
// 		leftError = targetLeft - avg_l();
// 		rightError = targetRight - avg_r();
		
// 		derivLeft = leftError - prevLeftError;
// 		derivRight = rightError - prevRightError;

// 		leftPower = (kP * leftError) + (kD * derivLeft);
// 		rightPower = (kP * leftError) + (kD * derivLeft);

// 		if(leftPower > (12000 * limit)) {
// 			leftPower = 12000 * limit;
// 		}
// 		if(rightPower > (12000 * limit)) {
// 			rightPower = 12000 * limit;
// 		}

// 		LeftDT.move_voltage(leftPower);
// 		RightDT.move_voltage(rightPower);

// 		prevLeftError = leftError;
// 		prevRightError = rightError;

// 		pros::delay(15);
// 	}while((fabs(targetLeft - avg_l()) > 10) && (fabs(targetRight - avg_r()) > 10));

// 	LeftDT.move_voltage(0);
// 	RightDT.move_voltage(0);
// 	LeftDT.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
// 	RightDT.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
// 	LeftDT.brake();
// 	RightDT.brake();
// }

double avg_r() {
	return (fabs(FR.get_position())+fabs(MR.get_position())+fabs(BR.get_position()) / 3);
}
double avg_l() {
	return (fabs(FL.get_position())+fabs(ML.get_position())+fabs(BL.get_position()) / 3);
}


// void drive(int ticks) {
// 	reset_encoder();

// 	double error = 0;
// 	double prevError = 0;
// 	double integral = 0;
// 	double derivative = 0;
// 	double power = 0;
// 	double kP = 15;
// 	double kI = 0;
// 	double kD = 0;
// 	// double kD = 0.15;
// 	double target = average_encoders() + ticks;
	
// 	while(fabs(target - average_encoders()) > 25) {
// 		error = target - average_encoders();
// 		prevError = error;
// 		integral = integral + error*0.015;
// 		derivative = error*0.015 - prevError;
// 		if(error == 0 || fabs(target - average_encoders()) > 50) {
//             integral = 0;
//         }
// 		power = (error * kP) + (integral * kI) + (derivative*kD);

// 		FR.move_velocity(power);
// 		FL.move_velocity(power);
// 		// MR.move_velocity(-(power));
// 		// ML.move_velocity((power));
// 		BR.move_velocity(power);
// 		BL.move_velocity(power);
// 		pros::delay(15);
// 	}
// 	// ML.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	// MR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
// 	FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);

//     // ML.move_voltage(0);
//     BL.move_voltage(0);
//     // MR.move_voltage(0);
// 	FL.move_voltage(0);
//     FR.move_voltage(0);
//     BR.move_voltage(0);
// }


// void pivot(double angle) {
// 	//theta = total turn
// 	//angle = target angle (0 to 360 degrees)
// 	//startAngle = current angle of robot -inf < degrees < inf
// 	double theta = 0;
// 	int startAngle = inertial.get_rotation();
// 	//get relative to (0 to 360 degrees)
// 	startAngle = startAngle % 360;
// 	//two cases
// 	if(startAngle > 180) {
// 		theta = 360 - startAngle;
// 		theta += angle;
// 		if(theta >= 180) {
// 			//if the turn is bigger than 180, impractical - turn the other way
// 			theta = 360 - theta;
// 			theta*=-1;
// 		}
// 	}
// 	else if(startAngle <= 180) {
// 		theta = (theta + startAngle) * -1;
// 		theta+=angle;
// 		if(theta >= 180) {
// 			//if the turn is bigger than 180, impractical - turn the other way
// 			theta = 360 - theta;
// 			theta*=-1;
// 		}
// 	}
// 	//theta is final how much to turn
// 	double error = 0;
// 	double prevError = 0;
// 	double integral = 0;
// 	double derivative = 0;
// 	double power = 0;
// 	double kP = 5;
// 	double kI = 0;
// 	double kD = 0.8;
// 	double target = inertial.get_rotation() + theta;
// 	while(fabs(target - inertial.get_rotation()) > 0.2) {
// 		error = target - inertial.get_rotation();
// 		prevError = error;
// 		integral = integral + error*0.015;
// 		derivative = error*0.015 - prevError;
// 		if(error == 0 || fabs(target - inertial.get_rotation()) > 0.5) {
//             integral = 0;
//         }
// 		power = (error * kP) + (integral * kI) + (derivative*kD);
// 		power*=0.9;
// 		FR.move_velocity(power);
// 		FL.move_velocity(power);
// 		// MR.move_velocity(power);
// 		// ML.move_velocity(power);
// 		BR.move_velocity(power);
// 		BL.move_velocity(power);
// 		pros::delay(15);
// 	}
// 	FR.move_voltage(0);
//     FL.move_voltage(0);
//     // MR.move_voltage(0);
// 	// ML.move_voltage(0);
//     BR.move_voltage(0);
// 	BL.move_voltage(0);
// }


		

		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
		// 	IIR.move_voltage(-10000);
		// 	pros::delay(60);
		// 	IIR.brake();
		// }

		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
		// 	turn(90);
		// }

		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
		// 	turn(359);
		// }
	
		// //Driver Toggle
		// if(flyState) {
		// 	setFlywheelRPM(420);
		// }
		// else {
		// 	setFlywheelRPM(0);
		// }