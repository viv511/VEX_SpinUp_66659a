#include "main.h"
#include "cmath"
#include "globals.h"
#include "variables.h"
#include <queue>
#include "fly.h"

int maxVolt = 12000;
int maxRPM = 3000;

pros::Mutex rpmMutex;
double target_rpm = 0;
double threshold = 25;

void setFlywheelRPM(double targetRPM) {
    rpmMutex.take();
    target_rpm = targetRPM;
    rpmMutex.give();
}

double getFlywheelRPM() {
    double tRPM;
    rpmMutex.take();
    tRPM = target_rpm;
    rpmMutex.give();
    return tRPM;
}

//--------------------------// FlyWheel //--------------------------//	

bool flyLast = false;

void flySpeed() {
	// int kP = 15;
	Fly.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	while(true) {
		//Toggle Logic
		if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && !flyLast) {
			flyState = !flyState;
			flyLast = true;
		}
		else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))) {
			flyLast = false;
		}

		//Get flywheel target speed
		float target_speed = getFlywheelRPM();
		float holdPower =  target_speed * maxVolt/maxRPM;
	
		//Get current speed
		currentSpeed = SMA_Filter(6 * Fly.get_actual_velocity());

		//Get error
		// float flyError = target_speed - currentSpeed; 

		if(target_speed == 0) {
			Fly.move_voltage(0);
			readyShoot = false;
		}
		else if(fabs(currentSpeed - target_speed) < threshold) {
			Fly.move_voltage(holdPower + 250);
			readyShoot = true;
		}
		else {
			if(currentSpeed < target_speed) {
				Fly.move_voltage(12000);
				readyShoot = false;
			}
			else{
				Fly.move_voltage(0);
				readyShoot = false;
			}
		}
		
		// if(shootingFunc) {
		// std::cout << currentSpeed << "," << target_speed << "\n";
		// }
		// pros::lcd::print(6, "error: %f\n", flyError);
		        pros::lcd::print(5, "Fly: %f\n", currentSpeed);

		pros::delay(20);
	}
}

//--------------------------// Filter //--------------------------//	
std::queue<double> smaData;
//Number of elements to average
int window = 4;
//Running sum
double windowTotal = 0;

double SMA_Filter(double rawData) {
	//Add to the running sum
	windowTotal += rawData;

	//Once the queue gets filled up
	if(smaData.size() >= window) {
		//Remove first num from queue & sum, then add new num (essentially shifting the queue)
		windowTotal -= smaData.front();
		smaData.pop();
	}
	//Add to the queue
	smaData.push(rawData);
	
	//Apply average
	double smoothedData = (windowTotal / smaData.size());
	return smoothedData;
}

// #include "main.h"
// #include "cmath"
// #include "globals.h"
// #include "variables.h"
// #include <queue>
// #include "fly.h"

// pros::Mutex rpmMutex;
// double target_rpm = 0;
// double threshold = 150;

// void setFlywheelRPM(double targetRPM) {
//     rpmMutex.take();
//     target_rpm = targetRPM;
//     rpmMutex.give();
// }

// double getFlywheelRPM() {
//     double tRPM;
//     rpmMutex.take();
//     tRPM = target_rpm;
//     rpmMutex.give();
//     return tRPM;
// }

// //--------------------------// FlyWheel //--------------------------//	

// bool flyLast = false;
// bool inTarget = false;

// void flySpeed() {
// 	double currentSpeed = 0;
// 	double error = 0;
// 	double flyPower = 0;
// 	float kP = 0.4;
// 	float kV = 0.04;

// 	Fly.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

// 	while(true) {
// 		if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && !flyLast) {
// 			flyState = !flyState;
// 			flyLast = true;
// 		}
// 		else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))) {
// 			flyLast = false;
// 		}

// 		float target_speed = getFlywheelRPM();
// 		float goalRPM = target_speed/600 * 3000;
// 		float holdPower = goalRPM * 12000/3000;
// 		// float holdPower = target_speed * 4;


// 		// if(inTarget) {
// 		currentSpeed = SMA_Filter(6 * Fly.get_actual_velocity());
// 		// }
// 		// else {
// 		// 	currentSpeed = 6 * Fly.get_actual_velocity();
// 		// }

// 		if(target_speed == 0) {
// 			Fly.move_voltage(0);
// 			inTarget = false;
// 		}
// 		else if(fabs(currentSpeed - target_speed) < threshold) {
// 			inTarget = true;

// 			error = holdPower - currentSpeed;
// 			flyPower = error * kP + holdPower * kV;

// 			Fly.move_voltage(flyPower);
// 			controller.rumble(".");
// 		}
// 		else {
// 			if(currentSpeed < target_speed) {
// 				Fly.move_voltage(12000);
// 				inTarget = false;
// 			}
// 			else{
// 				Fly.move_voltage(0);
// 				inTarget = false;
// 			}
// 		}
		
// 		pros::delay(20);


// 		if(flyState == true) {
// 			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){ 
// 				setFlywheelRPM(1600);
// 			}
// 			else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
// 				setFlywheelRPM(2600);
// 			}
// 			else {
// 				setFlywheelRPM(2000);
// 			}
// 		}
// 		else {
// 			setFlywheelRPM(0);
// 		}
// 	}
// }

// //--------------------------// Filter //--------------------------//	
// std::queue<double> smaData;
// //Number of elements to average
// int window = 4;
// //Running sum
// double windowTotal = 0;

// double SMA_Filter(double rawData) {
// 	//Add to the running sum
// 	windowTotal += rawData;

// 	//Once the queue gets filled up
// 	if(smaData.size() >= window) {
// 		//Remove first num from queue & sum, then add new num (essentially shifting the queue)
// 		windowTotal -= smaData.front();
// 		smaData.pop();
// 	}
// 	//Add to the queue
// 	smaData.push(rawData);
	
// 	//Apply average
// 	double smoothedData = (windowTotal / smaData.size());
// 	return smoothedData;
// }