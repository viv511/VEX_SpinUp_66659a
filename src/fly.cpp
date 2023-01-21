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
double threshold = 20;

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
	float flyPower = 0;
	int kP = 200;
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
		float flyError = target_speed - currentSpeed; 

		if(target_speed == 0) {
			flyPower = 0;

			readyShoot = false;
		}
		else {
			if(fabs(flyError) < 50) {
				flyPower = 2*holdPower;

				readyShoot = true;
			}
			else {
				flyPower = kP * flyError;

				readyShoot = false;
			}
		}
		
		Fly.move_voltage(flyPower);

		printf("%.2f\n", currentSpeed);

		pros::lcd::print(5, "Fly: %f\n", currentSpeed);
		pros::lcd::print(6, "power: %f\n", flyPower);

		pros::delay(10);
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

// int maxVolt = 12000;
// int maxRPM = 3000;

// pros::Mutex rpmMutex;
// double target_rpm = 0;
// double threshold = 25;

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
// void flySpeed() {
// 	bool flyLast = false;
// 	double gainVal = 0.07;

// 	float tbh = 0;
// 	float flyPower = 0;
// 	float prevFlyError = 0;

// 	Fly.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

// 	while(true) {
// 		//Toggle Logic
// 		if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && !flyLast) {
// 			flyState = !flyState;
// 			flyLast = true;
// 		}
// 		else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))) {
// 			flyLast = false;
// 		}

// 		//Get flywheel target speed
// 		float target_speed = getFlywheelRPM();
// 		//Get current speed
// 		currentSpeed = 6 * Fly.get_actual_velocity();
// 		//Get error
// 		float flyError = target_speed - currentSpeed;
	
// 		//Implement TBH Control
// 		flyPower += (gainVal * flyError);

// 		if(std::signbit(flyError) != std::signbit(prevFlyError)) {
// 			flyPower = 0.5 * (flyPower + tbh);
// 			tbh = flyPower;
// 			prevFlyError = flyError;
// 		}

// 		Fly.move_voltage(flyPower);

// 		if(fabs(flyError) < threshold) {
// 			readyShoot = true;
// 		}
// 		else {
// 			readyShoot = false;
// 		}
		
// 		pros::lcd::print(5, "Fly: %f\n", currentSpeed);
// 		pros::lcd::print(6, "Flypower: %f\n", flyPower);

// 		pros::delay(10);
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