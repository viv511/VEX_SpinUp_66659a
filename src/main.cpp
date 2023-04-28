#include "main.h"

#include "globals.h"
#include "odom.h"
#include "fly.h"
#include "driveauto.h"
#include "waypoint.h"
#include "pid.h"
#include "purepursuit.h"
// #include "matome.h"

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
	blooper.set_value(false);
	pros::lcd::initialize();

	inertial.reset();
	pros::delay(2200);
	// while(inertial.is_calibrating()) {
	// 	pros::delay(5);
	// }
	inertial.set_rotation(0);

	// setPIDvalues();
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
	setHold();
	inertial.set_rotation(0);

	rightAuto();
	// leftAuto();
}
/*

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
	endgame1.set_value(false);
	inertial.set_rotation(0);
	setCoast();
	blooper.set_value(true);
	setToggle(false);
	setFlywheelRPM(0);

	long long controllerTime = 0;

	float driveSense = 1;
	float turnSense = 0.8;
	float leftPower = 0;
	float rightPower = 0;

	bool bloopState = true;
	bool bloopLast = true;

	while(true) {
		// *---*---*---*---*---*---*--CONTROLLER AND DRIVE--*---*---*---*---*---*---*---*---*
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
						pros::delay(200);
						endgame1.set_value(false);
						controller.rumble(".");
					}
				}
			}
		}

		// *---*---*---*---*---*--INTAKE, INDEXER, AND ROLLER--*---*---*---*---*---*---*---*
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
				IIR.move_voltage(-12000);
				pros::delay(100);
				IIR.move_voltage(0);

				pros::delay(20);

				IIR.move_voltage(-12000);
				pros::delay(100);
				IIR.move_voltage(0);

				pros::delay(20);

				IIR.move_voltage(-12000);
				pros::delay(300);
				IIR.move_voltage(0);
			}
			else {
				//Index
				IIR.move_voltage(-12000);
			}
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
		if(getToggle()) {
			setFlywheelRPM(2400);
		}
		else {
			setFlywheelRPM(0);
		}

		if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) && !bloopLast) {
			bloopState = !bloopState;
			bloopLast = true;
		}
		else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)))) {
			bloopLast = false;
		}

		if(bloopState) {
			blooper.set_value(true);
		}
		else {
			blooper.set_value(false);
		}

		

		// *---*---*---*---*---*--DEBUGGING UTILS--*---*---*---*---*---*---*---*
		if(!(controllerTime % 10)) {
			controller.print(0, 0, "RPM: %.2f", Fly.get_actual_velocity()*6);
		}
		
		pros::delay(10);
		controllerTime++;
	}
}

void leftAuto() {
	move(1, 1);
	Fly.move_velocity(-400);
	IIR.move_voltage(-12000);
	pros::delay(130);
	IIR.move_voltage(0);
	Fly.move_velocity(0);
	IIR.move_voltage(12000);

	setFlywheelRPM(3100);

	IIR.move_voltage(12000);
	move(-3.5, 1);
	turn(-70);
	IIR.move_voltage(0);
	
	move(45, 1);
	turn(70);
	move(-42, 1);
	turn(-8);
	shoot(3100);

	turn(45);
	IIR.move_voltage(-12000);
	move(20, 1);
	IIR.move_voltage(0);
	IIR.move_voltage(12000);
	move(23, 0.6);
	move(-51, 1);
	IIR.move_voltage(0);
	turn(-52);

	shoot(3100);
}
 
void rightAuto() {
	move(22, 1);
	turn(90);
	move(8, 1);
	IIR.move_voltage(-12000);
	pros::delay(150);
	IIR.move_voltage(0);

	IIR.move_voltage(12000);
	setFlywheelRPM(3100);
	move(-8, 1);
	turn(124);
	move(28, 1);
	turn(-94);
	move(-10, 1);
	IIR.move_voltage(0);

	shoot(3100);
	move(10, 1);
	turn(106);

	IIR.move_voltage(12000);
	move(32, 1);
	move(-28, 1);
	turn(-100);
	move(-10, 1);
	IIR.move_voltage(0);
	shoot(3100);
}