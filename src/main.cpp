#include "main.h"
#include "globals.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cmath>


// // Create grapher
// std::shared_ptr<graphy::AsyncGrapher> grapher(new graphy::AsyncGrapher("Flywheel Velocity vs. Time"));

// // Add data types
// grapher->addDataType("Desired Vel", COLOR_ORANGE);
// grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);

// // Start grapher task
// grapher->startTask();
//--------------------------// Odometry //--------------------------//			

//Utility
bool PTO = false;
constexpr float PI = 3.141592;
constexpr float degreesToRadians = PI/180;
constexpr float radiansToDegrees = 180/PI;

//Distance from center
constexpr float LR_DIST = 3.25;
constexpr float B_DIST = 3.5;

//Diameter of wheels
constexpr float LR_DIAMETER = 2.8;
constexpr float B_DIAMETER = 2.8;

//Ticks per rotation
constexpr float LR_TICKS = 36000.0;
constexpr float B_TICKS = 360.0;

//L + Ratio (jk)
constexpr float LR_RATIO = (PI * LR_DIAMETER)/LR_TICKS; 
constexpr float B_RATIO = (PI * B_DIAMETER)/B_TICKS;

int leftCurrent = 0;
int rightCurrent = 0;
int backCurrent = 0;
int leftLast = 0;
int rightLast = 0;
int backLast = 0;

float rightChange;
float leftChange;
float backChange;
float angleChange;

float absoluteAngle = 0;
float absoluteLeft = 0;
float absoluteRight = 0;
float absoluteBack = 0;

float x_start = 0;
float y_start = 0;
float angle_start = 0;

float x_local = 0;
float y_local = 0;
float delta_x_global = 0;
float delta_y_global = 0;
float x_global = x_start;
float y_global = y_start;
float offsetCoord = absoluteAngle + (angleChange / 2);

void odometry() {	

	while(true) {
		leftCurrent = leftEncoder.get_position();
		rightCurrent = rightEncoder.get_position() * -1;
		backCurrent = backEncoder.get_value();	
		
		//Calculate the change in encoder since last cycle, convert to inches
		leftChange = (leftCurrent - leftLast);
		rightChange = (rightCurrent - rightLast);
		backChange = (backCurrent - backLast);

		//Ticks --> Inches
		leftChange *= LR_RATIO;
		rightChange *= LR_RATIO;
		backChange *= B_RATIO;

		//Store last values
		leftLast = leftCurrent;
		rightLast = rightCurrent;
		backLast = backCurrent;

		//Find the angle change from the last cycle (Radians)
		angleChange = angle_start + ((leftChange - rightChange) / (LR_DIST + LR_DIST));

		//Calculate absolute orientation + absolute left, right, and back encoder change
		absoluteLeft += leftChange;
		absoluteRight += rightChange;
		absoluteBack += backChange;	
		absoluteAngle += angleChange;
	
		
		if(angleChange == 0) {
			x_local = backChange;
			y_local = rightChange;
		}
		else {
			//need to find radius, then multiply by 2 sin (deltatheta/2)
			x_local = ((backChange / angleChange) + B_DIST) * 2 * sin(angleChange / 2.0);
			y_local = ((rightChange / angleChange) + LR_DIST) * 2 * sin(angleChange / 2.0);
		}

		//need to translate local --> global
		//check
		offsetCoord = absoluteAngle - (angleChange / 2);
		
		delta_x_global = (y_local * cos(offsetCoord)) - (x_local * sin(offsetCoord));
		delta_y_global = (y_local * sin(offsetCoord)) + (x_local * cos(offsetCoord));


		x_global += delta_x_global;
		y_global += delta_y_global;


		// pros::lcd::print(1, "L: %f\n", absoluteLeft);
		// pros::lcd::print(2, "R: %f\n", absoluteRight);
		// pros::lcd::print(3, "B: %f\n", absoluteBack);
		// pros::lcd::print(4, "(rad) Angle: %f\n", absoluteAngle);
		// pros::lcd::print(5, "X: %f\n", x_local);
		// pros::lcd::print(6, "Y: %f\n", y_local);

		pros::delay(10);
	}
}

static double target_rpm = 0;
static double threshold = 20;
static bool is_ready_to_fire = false;

pros::Mutex flywheel_target_m; // mutex for target rpm
pros::Mutex flywheel_ready_m; // mutex for ready to fire

void set_target_rpm(double targetrpm) {
    flywheel_target_m.take(); //take the mutex, letting others know we are accessing the target rpm
    target_rpm = targetrpm; //set the target rpm
    flywheel_target_m.give(); //give the mutex back
}

double get_target_rpm() {
    double targetrpm;
    flywheel_target_m.take(); //take the mutex, letting others know we are accessing the target rpm
    targetrpm = target_rpm; //get the target rpm
    flywheel_target_m.give(); //give the mutex back
    return targetrpm;
}

void set_flywheel_ready(bool ready) {
    flywheel_ready_m.take(); //take the mutex, letting others know we are accessing the ready to fire variable
    is_ready_to_fire = ready; //set the ready to fire variable
    flywheel_ready_m.give(); //give the mutex back
}

bool get_flywheel_ready() {
    bool ready;
    flywheel_ready_m.take(); //take the mutex, letting others know we are accessing the ready to fire variable
    ready = is_ready_to_fire; //get the ready to fire variable
    flywheel_ready_m.give(); //give the mutex back
    return ready;
}

bool flyState = false;
bool flyLast = false;
//--------------------------// FlyWheel //--------------------------//	
void flySpeed() {
	

	while(true) {
		if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && !flyLast) {
			flyState = !flyState;
			flyLast = true;
		}
		else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))) {
			flyLast = false;
		}

		double target_speed = get_target_rpm();
        int goalRPM = target_speed/600 * 3000;
        int holdPower = goalRPM * 12000/3000;
    
        double current_speed = Fly.get_actual_velocity(); // get the current rpm. I would recommend using something like a Simple Moving Average filter to smooth out the rpm

		if(target_speed == 0) {
			Fly.move_voltage(0); // if the target rpm is 0, stop the flywheel
            set_flywheel_ready(false); // the flywheel is not ready to fire
		}
		else if(fabs(current_speed - target_speed) < threshold) {
			Fly.move_voltage(holdPower);
			set_flywheel_ready(true);
			controller.rumble(".");
		}
		else {
            Fly.move_voltage(current_speed < target_speed ? 12000 : 0); // if the flywheel is not within the threshold, bang bang the flywheel speed
            set_flywheel_ready(false); // set the flywheel ready to fire variable to false
        }

		pros::delay(10);
	}


	// bool flyState = false;
	// bool flyLast = false;

	// while(true) {
	
	// 	if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && !flyLast) {
	// 		flyState = !flyState;
	// 		flyLast = true;
	// 	}
	// 	else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))) {
	// 		flyLast = false;
	// 	}

	// 	if(flyState == true) {
	// 		int goalSpeed = 450;
	// 		int goalRPM = goalSpeed/600 * 3000;
	// 		//12 volts = max Voltage, 3000 rpm is max rpm
	// 		//hold power = goalRPM * maxV/maxRPM
	// 		int holdPower = goalRPM * 12000/3000;

	// 		if(Fly.get_actual_velocity() < (goalSpeed - 20)) {
	// 			Fly.move_voltage(12000);
	// 		}
	// 		else {
	// 			controller.rumble(".");
	// 			Fly.move_voltage(holdPower);
	// 		}
	// 	}
	// 	else {
	// 		Fly.move_voltage(0);
	// 	}
		
	// 	std::cout << Fly.get_actual_velocity() << std::endl;			
	// 	pros::lcd::print(7, "Fly: %f\n", Fly.get_actual_velocity());
	// 	pros::delay(30);
	// }
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
	endgame1.set_value(true);
	pros::lcd::initialize();
	inertial.reset();
	while(inertial.is_calibrating()) {
	pros::delay(10);
	}
	inertial.set_rotation(0);

	// backEncoder.reset();
	// leftEncoder.reset();
	// rightEncoder.reset();

	// leftEncoder.set_position(0);
	// rightEncoder.set_position(0);

	// pros::Task odom (odometry);
	pros::Task flywheelstuff (flySpeed);
	Fly.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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
	rightAuto();
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
	PTO = true;
	Fly.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	Indexer.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	while(true) {
			pros::delay(10);
			pros::lcd::print(3, "Inertial: %f\n", inertial.get_rotation());


			// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
			// 	rightAuto();
			// }
			// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			// 	soloAuto();
			// }

			if(flyState == true) {
				set_target_rpm(450);
			}
			else {
				set_target_rpm(0);
			}

			int a4 = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
			int a3 = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
			FL.move(a4 + a3);
			FR.move(a4 - a3);
			BL.move(a4 + a3);
			BR.move(a4 - a3);

			//endgame
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
				if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
					endgame1.set_value(false);
				}
			}

			if(PTO == false) {
				ML_intake.move(a4 + a3);
				MR_intake.move(a4 - a3);
			}
			else{
				if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
					ML_intake.move_voltage(-6000);
					MR_intake.move_voltage(6000);					
				}
				if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
					ML_intake.move_voltage(-12000);
					MR_intake.move_voltage(12000);
				}
				else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
					MR_intake.move_voltage(-12000);
					ML_intake.move_voltage(12000);
				}
				else if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) || (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) || (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) || (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))){
					ML_intake.move_voltage(0);
					MR_intake.move_voltage(0);
				}				
				
			}			
			
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
				Indexer.move_voltage(12000);
			}
			else{
				Indexer.move_voltage(0);
			}


		}
}

void halfRoll() {
	ML_intake.move_voltage(6000);
	MR_intake.move_voltage(-6000);
	pros::delay(140);
	ML_intake.brake();
	MR_intake.brake();
}

void tripleShot(double speed) {
	set_target_rpm(speed);
	pros::delay(2000);

	for(int i=0; i<2; i++) {
		Indexer.move_voltage(12000);
		pros::delay(200);
		Indexer.brake();

		pros::delay(200);
	}

	Indexer.move_voltage(12000);
	pros::delay(600);
	Indexer.move_voltage(0);
	set_target_rpm(0);
}

void soloAuto() {
	move_encoder(1100, 300);
	turn(90);
	pros::delay(100);	
	forward(400);

	ML_intake.move_voltage(7000);
	MR_intake.move_voltage(-7000);
	pros::delay(400);
	ML_intake.brake();
	MR_intake.brake();

	move_encoder(50, -200);
	pros::delay(50);
	turn(130);

	pros::delay(50);
	set_target_rpm(470);
	ML_intake.move_voltage(-12000);
	MR_intake.move_voltage(12000);
	move_encoder(1600, 200);
	ML_intake.move_voltage(0);
	MR_intake.move_voltage(0);
	move_encoder(2000, 400);

	ML_intake.move_voltage(-12000);
	MR_intake.move_voltage(12000);
	move_encoder(300, -300);
	turn(-85);
	ML_intake.move_voltage(0);
	MR_intake.move_voltage(0);
	Indexer.move_voltage(12000);
	pros::delay(300);
	Indexer.brake();

	pros::delay(800);

	Indexer.move_voltage(12000);
	pros::delay(300);
	Indexer.brake();
	set_target_rpm(0);
	turn(90);
	move_encoder(1000, 400);
	turn(-45);

	move_encoder(2000, 400);
	pros::delay(50);
	turn(90);
	pros::delay(50);
	move_encoder(1800, 500);
	pros::delay(50);
	turn(-90);

	forward(600);

	ML_intake.move_voltage(7000);
	MR_intake.move_voltage(-7000);
	pros::delay(400);
	ML_intake.brake();
	MR_intake.brake();
}

void rightAuto() {
	move_encoder(1100, 300);
	turn(90);
	pros::delay(100);
	forward(400);

	ML_intake.move_voltage(7000);
	MR_intake.move_voltage(-7000);
	pros::delay(250);
	ML_intake.brake();
	MR_intake.brake();

	move_encoder(100, -200);

	pros::delay(100);
	turn(90);
	pros::delay(50);
	move_encoder(2000, 300);
	pros::delay(50);
	turn(90);
	move_encoder(300, -400);

	ML_intake.move_voltage(-12000);
	MR_intake.move_voltage(12000);
	move_encoder(2100, 300);

	turn(-135);

	set_target_rpm(460);
	pros::delay(2500);
	ML_intake.move_voltage(0);
	MR_intake.move_voltage(0);

	Indexer.move_voltage(12000);
	pros::delay(200);
	Indexer.brake();

	pros::delay(800);
	Indexer.move_voltage(12000);
	pros::delay(1000);
	Indexer.brake();

	set_target_rpm(0);
}

void leftAuto() {
	forward(300);
	halfRoll();
	move_encoder(80, -200);
	turn(-90);

	move_encoder(2100, 400);
	pros::delay(100);
	
	turn(90);
	pros::delay(100);
	move_encoder(1000, -400);
	pros::delay(50);
	turn(-32);
	pros::delay(50);
	move_encoder(750, -300);
	
	set_target_rpm(460);
	pros::delay(2500);

	Indexer.move_voltage(12000);
	pros::delay(300);
	Indexer.brake();

	pros::delay(800);
	Indexer.move_voltage(12000);
	pros::delay(1000);
	Indexer.brake();

	set_target_rpm(0);
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
	double kP = 5;
	double kI = 0;
	double kD = 0.8;

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
		// MR.move_velocity(power);
		// ML.move_velocity(power);
		BR.move_velocity(power);
		BL.move_velocity(power);
		pros::delay(15);
	}
	
	FR.move_voltage(0);
    FL.move_voltage(0);
    // MR.move_voltage(0);
	// ML.move_voltage(0);
    BR.move_voltage(0);
	BL.move_voltage(0);
	
}

// void tbh(double speed) {
// 	double power = 0;
// 	double prevPower = 0;
// 	double error = 0;
// 	double gain = 1;
// 	while(fabs(speed - Fly.get_actual_velocity()) > 1) {
// 		error = speed - Fly.get_actual_velocity();
// 		power = prevPower + (error) * gain;
// 		prevPower = power;
// 		Fly.move_voltage(power);
// 		pros::delay(10);
// 	}

void forward(double sec) {
	FL.move_velocity(100);
	FR.move_velocity(-100);
	BL.move_velocity(100);
	BR.move_velocity(-100);
	pros::delay(sec);
	FL.move_velocity(0);
	FR.move_velocity(0);
	BL.move_velocity(0);
	BR.move_velocity(0);
}

void turn(double angle) {
	double error = 0;
	double prevError = 0;
	// double integral = 0;
	double derivative = 0;
	double power = 0;
	double kP = 5;
	// double kI = 0;
	double kD = 0.8;

	double target = inertial.get_rotation() + angle;
	
	while(fabs(target - inertial.get_rotation()) > 0.2) {
		error = target - inertial.get_rotation();
		prevError = error;
		// integral = integral + error*0.015;
		derivative = error*0.015 - prevError;
		// if(error == 0 || fabs(target - inertial.get_rotation()) > 0.5) {
        //     integral = 0;
        // }
		// power = (error * kP) + (integral * kI) + (derivative*kD);
		power = (error * kP) + (derivative*kD);

		FR.move_velocity(power);
		FL.move_velocity(power);
		// MR.move_velocity(power);
		// ML.move_velocity(power);
		BR.move_velocity(power);
		BL.move_velocity(power);
		pros::delay(10);
	}
	FL.move_voltage(0);
    // ML.move_voltage(0);
    BL.move_voltage(0);
    FR.move_voltage(0);
    // MR.move_voltage(0);
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
		// MR.move_velocity(-(power));
		// ML.move_velocity((power));
		BR.move_velocity(-(power));
		BL.move_velocity((power));
		pros::delay(15);
	}
	// ML.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// MR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    // ML.move_voltage(0);
    BL.move_voltage(0);
    // MR.move_voltage(0);
	FL.move_voltage(0);
    FR.move_voltage(0);
    BR.move_voltage(0);
}

void move_encoder(double ticks, double speed) {
	reset_encoder();
	
	FR.move_velocity(-speed);
	FL.move_velocity(speed);
	BR.move_velocity(-speed);
	BL.move_velocity(speed);

	while(fabs(average_encoders())<ticks){
			pros::delay(2);
	}

	// ML.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	BL.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	// MR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	BR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	FR.set_brake_mode(E_MOTOR_BRAKE_HOLD);
	FL.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    // ML.move_voltage(0);
    BL.move_voltage(0);
    // MR.move_voltage(0);
	FL.move_voltage(0);
    FR.move_voltage(0);
    BR.move_voltage(0);

}
void reset_encoder() {
	FL.tare_position();
	BL.tare_position();
	FR.tare_position();
	BR.tare_position();
}
double average_encoders() {
	return (fabs(FR.get_position())+fabs(FL.get_position())+fabs(BL.get_position()) + fabs(BR.get_position()))/ 4;
}

double avg_r() {
	// return (fabs(FR.get_position()) + fabs(BR.get_position()) + fabs(MR.get_position())) / 3;
	return (fabs(FR.get_position()) + fabs(BR.get_position())) / 2;
}

double avg_l() {
	return (fabs(FL.get_position()) + fabs(BL.get_position())) / 2;
}

void autoAim(double rX, double rY, double gX, double gY) {
	float theta = atan2((gY-rY), (gX-rX));
	turn(theta);
}

