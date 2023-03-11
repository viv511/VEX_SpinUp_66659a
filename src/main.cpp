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
#include <string>

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
	endgame1.set_value(false);
	pros::lcd::initialize();

	// inertial.reset(true);
	inertial.reset();
	while(inertial.is_calibrating()) {
		pros::delay(5);
	}
	inertial.set_rotation(0.00);

	// setPIDvalues();
	backEncoder.reset();
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
	autonThreshold = true;
	LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

	// walter();
	RollerSide();
	// AWP();
	// NonRoller();
	// regionals();
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
	autonThreshold = false;
	flyState = false;
	setFlywheelRPM(0);
	LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
	RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

	float leftPower = 0;
	float rightPower = 0;

	float driveSense, turnSense;

	long long controllerTime = 0;

	bool defenseLast = false;
	bool defenseState = false;

	std::string zoom = "";

	bool speedLast = false;
	bool speedState = false;

	std::string driveState = "";
	
	float curAng = inertial.get_rotation();
	float prevAng = curAng;

	while(true) {
		// *---*---*---*---*---*---*--CONTROLLER AND DRIVE--*---*---*---*---*---*---*---*---*
		if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) && !defenseLast) {
			defenseState = !defenseState;
			defenseLast = true;
		}
		else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)))) {
			defenseLast = false;
		}
		
		if(defenseState) {
			driveSense = 1;
			turnSense = 0.8;
			driveState = "ON";
		}
		else {
			driveSense = 0.8;
			turnSense = 0.35;
			driveState = "OFF";
		}

		// curAng = inertial.get_rotation();
		//Bind longitude and latitude from -1 to 1 from the controller
		float lon = ((controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127.0) * driveSense;
		float lat = ((controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127.0) * turnSense;

		//Find scaled bounded maximum for drivetrain %'s
		float mag = fmax(1.0, fmax(fabs(lon + lat), fabs(lon - lat)));

		//-1.0 <--  0.0 --> 1.0 scale to voltage (-12000 <-- 0 --> 12000 mV)
		leftPower = ((lon + lat) / mag) * 600;
		rightPower = ((lon - lat) / mag) * 600;

		//Assign power
		LeftDT.move_velocity(leftPower);
		RightDT.move_velocity(rightPower);
	
		// *---*---*---*---*---*---*---*--ENDGAME--*---*---*---*---*---*---*---*---*---*
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
				if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
					if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
						endgame1.set_value(true);
						controller.rumble(".");
					}
				}
			}
		}
		
		// *---*---*---*---*---*--INTAKE, INDEXER, AND ROLLER--*---*---*---*---*---*---*---*
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			//Index
			IIR.move_voltage(-12000);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			IIR.move_voltage(9000);
		}
		else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			IIR.move_voltage(12000);
		}
		else{
			IIR.move_voltage(0);
		}

		
		

		// *---*---*---*---*---*--FLYWHEEL CONTROLLER--*---*---*---*---*---*---*---*
		if(flyState == true) {
			if(speedState) {
				setFlywheelRPM(2150);
				zoom = "-L";
			}
			else {
				setFlywheelRPM(2200);
				zoom = "-N";
			}

			if(flyError < 20) {
				controller.rumble(".");
			}
		}
		else {
			setFlywheelRPM(0);
			zoom = "";
		}
		if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) && !speedLast) {
			speedState = !speedState;
			speedLast = true;
		}
		else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)))) {
			speedLast = false;
		}



		// *---*---*---*---*---*--DEBUGGING UTILS--*---*---*---*---*---*---*---*
		if(!(controllerTime % 5)) {
			std::string s = "";
			s = driveState + zoom;
			controller.print(0, 0, "Full power: %s", s);
			// controller.print(0, 0, "RPM: %.1f", getFlywheelRPM()-flyError);
			// controller.clear();
		}
		

		pros::lcd::print(2, "inertial: %f\n", inertial.get_rotation());
		

		pros::delay(10);
		controllerTime++;
	}
}

void waitShoot(int waitTime) {
	IIR.move_voltage(-12000);
	pros::delay(120);
	IIR.move_voltage(0);
	pros::delay(waitTime);

	IIR.move_voltage(-12000);
	pros::delay(140);
	IIR.move_voltage(0);
	pros::delay(waitTime+50);

	IIR.move_voltage(-12000);
	pros::delay(400);
	IIR.move_voltage(0);
}

void regionals() {
	//1st Volley
	flyState = true;
	setFlywheelRPM(2200);
	forwardPD(3.5, 1);
	IIR.move_voltage(12000);
	pros::delay(400);
	IIR.move_voltage(0);
	forwardPD(-6, 1);
	pivot(115);
	IIR.move_voltage(12000);
	forwardPD(14.5, 0.7);
	pivot(89);
	IIR.move_voltage(0);
	forwardPD(14, 1);
	IIR.move_voltage(12000);
	pros::delay(400);
	IIR.move_voltage(0);
	forwardPD(-10, 0.9);
	pivot(0);
	IIR.move_voltage(12000);
	forwardPD(-55, 1); 
	pivot(10);
	IIR.move_voltage(0);
	bucket(2200, 10, 4000, 100);

	//2nd Volley
	pivot(-45);
	IIR.move_voltage(12000);
	forwardPD(23, 0.8);
	pivot(-135);
	forwardPD(37, 0.7);
	pivot(-43);
	forwardPD(-8, 1);
	pros::delay(500);
	IIR.move_voltage(0);
	bucket(2200, 10, 4000, 100);
	controller.rumble("-");

	//3rd Volley
	forwardPD(6, 1);
	pivot(-135);
	forwardPD(28, 0.8);
	IIR.move_voltage(12000);
	forwardPD(31.5, 0.4);
	pros::delay(750);
	pivot(-180);
	forwardPD(3.8, 1);
	IIR.move_voltage(0);
	IIR.move_voltage(12000);
	pros::delay(80);
	IIR.move_voltage(0);
	forwardPD(-6, 1);
	pivot(-90);
	IIR.move_voltage(12000);
	forwardPD(-43, 0.9);
	IIR.move_voltage(0);
	pivot(-85);
	bucket(2200, 10, 4000, 100);

	//4th Volley
	pivot(-69);
	forwardPD(39, 1);
	IIR.move_voltage(12000);
	forwardPD(34, 0.6);
	pros::delay(50);
	forwardPD(-20.5, 1);
	pivot(-90);
	forwardPD(21.5, 1);
	forwardPD(2.5, 1);
	pros::delay(120);
	IIR.move_voltage(0);
	
	forwardPD(-13, 1);
	pivot(-180);
	IIR.move_voltage(12000);
	forwardPD(-43, 0.9); 
	IIR.move_voltage(0);
	pivot(-175);
	bucket(2200, 10, 4000, 100);
	pivot(139);

	//5th Volley
	IIR.move_voltage(12000);
	forwardPD(24, 0.9);
	pivot(45);
	forwardPD(33, 0.7);
	pivot(138);
	forwardPD(-8, 1);
	IIR.move_voltage(0);
	bucket(2200, 10, 4000, 100);
	forwardPD(8, 1);
	pivot(42);

	//6th Volley
	forwardPD(28, 0.7);	
	IIR.move_voltage(12000);
	forwardPD(26.5, 0.6);
	setFlywheelRPM(2200);
	pivot(93);
	forwardPD(-37, 1);
	IIR.move_voltage(0);
	// pivot(100);
	bucket(2200, 10, 4000, 100);

	//7th Volley
	pivot(115);
	forwardPD(41, 1);
	IIR.move_voltage(12000);
	forwardPD(31, 0.7);
	forwardPD(-12, 1);
	pivot(0);
	forwardPD(-40, 1);
	pivot(4);
	IIR.move_voltage(0);
	bucket(2200, 10, 4000, 100);

	//END
	pivot(0);
	forwardPD(51.5, 1);
	pivot(45);
	forwardPD(7, 1);
	// controller.rumble("-");
	endgame1.set_value(true);
}

void walter() {
	autonThreshold = false;
	flyState = true;
	setFlywheelRPM(3500);
	IIR.move_voltage(-12000);
	forwardPD(3.5, 1);
	pros::delay(50);
	forwardPD(-4.5, 1);
	IIR.move_voltage(0);
	pivot(-3);
	
	shoot(3, 3450, 4000, 5, 100);
	
	flyState = false;
	setFlywheelRPM(0);
}

void RollerSide() {
	autonThreshold = false;
	flyState = true;
	setFlywheelRPM(3050);
	IIR.move_voltage(-12000);
	forwardPD(3.5, 1);
	pros::delay(80);
	forwardPD(-4.5, 1);
	pivot(-132);
	// IIR.move_voltage(-12000);
	IIR.move_voltage(0);
	forwardPD(22, 0.7);
	IIR.move_voltage(12000);
	forwardPD(27.5, 0.6);
	pivot(-30);
	forwardPD(-10, 1);
	IIR.move_voltage(0);
	
	shoot(3, 3050, 4000, 5, 100);
	flyState = false;
	setFlywheelRPM(0);
	forwardPD(20, 1);
}

void AWP() {
	autonThreshold = false;
	flyState = true;
	setFlywheelRPM(3100);
	IIR.move_voltage(-12000);
	forwardPD(3.5, 1);
	pros::delay(80);
	forwardPD(-4.5, 1);
	pivot(-132);
	IIR.move_voltage(12000);
	IIR.move_voltage(0);
	forwardPD(22, 0.7);
	IIR.move_voltage(12000);
	forwardPD(27.5, 0.6);
	pivot(-30);
	forwardPD(-10, 1);
	IIR.move_voltage(0);
	
	shoot(3, 3100, 4000, 5, 100);

	IIR.move_voltage(12000);
	forwardPD(7, 1);
	pivot(-138);

	flyState = false;
	setFlywheelRPM(0);	

	forwardPD(76, 0.9);
	pivot(-87);

	forwardPD(9, 1);
	IIR.move_voltage(0);

	IIR.move_voltage(-12000);
	pros::delay(200);
	IIR.move_voltage(0);
}

void NonRoller() {
	autonThreshold = false;
	flyState = true;
	setFlywheelRPM(3000);
	IIR.move_voltage(12000);
	forwardPD(28, 0.5);
	pros::delay(500);
	pivot(-145);
	IIR.move_voltage(0);
	shoot(3, 3000, 4000, 5, 100);
	forwardPD(3, 1);
	pivot(135);

	forwardPD(35, 1);
	pivot(-175);
	forwardPD(8, 1);

	IIR.move_voltage(-12000);
	pros::delay(300);
	IIR.move_voltage(0);
}