#include "main.h"
#include "cmath"
#include "globals.h"
#include "variables.h"
#include <queue>
#include "fly.h"

const int rpmThreshold = 100;
int maxVolt = 12000;
int maxRPM = 3200;

pros::Mutex rpmMutex;
double target_rpm = 0;

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
	float kV = 3.6;
	float kS = 1200;
	float kP = 0;
	float kI = 0;

	float flyPower = 0;
	float flyError = 0;
	float flyIntegral = 0;
	float prevFlyError = flyError;

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

		//Get flywheel target speed and current speed
		float targetSpeed = getFlywheelRPM();
		currentSpeed = SMA_Filter(6 * Fly.get_actual_velocity());

		//Calculate error
		flyError = targetSpeed - currentSpeed;

		//Set speed
		if(targetSpeed == 0) {
			flyPower = 0;
		}
		else {
			if(flyError > rpmThreshold){
				flyPower = 12000;
			}
			else if(flyError < -rpmThreshold) {
				flyPower = 0;
			}
			else {	
				// flyIntegral += flyError;

				//Integral checks (zero crossing & outside useful range)
				// if(std::signbit(flyError) != std::signbit(prevFlyError)) {
				// 	flyIntegral = 0;
				// }
				// if(flyError > 20) {
				// 	flyIntegral = 0;
				// }
				
				// flyPower = (kV * targetSpeed) + (flyError * kP) + (flyIntegral * kI);
				flyPower = (kV * targetSpeed) + kS + (flyError * kP);
			}
		}
		
		if(flyPower < 0) {
			flyPower = 0;
		}
		if(flyPower > 12000) {
			flyPower = 12000;
		}

		Fly.move_voltage(flyPower);

		prevFlyError = flyError;

		printf("%.2f\n", currentSpeed);
		pros::lcd::print(5, "Fly: %f\n", currentSpeed);
		pros::lcd::print(6, "power: %f\n", flyPower);
		pros::delay(10);
	}
}

// #include "main.h"
// #include "cmath"
// #include "globals.h"
// #include "variables.h"
// #include <queue>
// #include "fly.h"

// const int rpmThreshold = 100;
// int maxVolt = 12000;
// int maxRPM = 3000;

// pros::Mutex rpmMutex;
// double target_rpm = 0;

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

// void flySpeed() {
// 	float kP = 20;
// 	float kI = 4;
// 	float flyPower = 0;
// 	float flyError = 0;
// 	float flyIntegral = 0;
// 	float prevFlyError = flyError;

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

// 		//Get flywheel target speed and current speed
// 		float targetSpeed = getFlywheelRPM();
// 		currentSpeed = SMA_Filter(6 * Fly.get_actual_velocity());

// 		//Calculate error
// 		flyError = targetSpeed - currentSpeed;

// 		//Base voltage for the target speed
// 		float holdPower = targetSpeed * maxVolt/maxRPM;

// 		//Set speed
// 		if(targetSpeed == 0) {
// 			flyPower = 0;
// 		}
// 		else if(flyError > rpmThreshold){
// 			flyPower = 12000;
// 			flyIntegral = 0;
// 		}
// 		else {	
// 			flyIntegral += flyError;

// 			//Integral checks (zero crossing & outside useful range)
// 			if(std::signbit(flyError) != std::signbit(prevFlyError)) {
// 				flyIntegral = 0;
// 			}
// 			if(flyError > 20) {
// 				flyIntegral = 0;
// 			}
			
// 			flyPower = holdPower + (flyError * kP) + (flyIntegral * kI);
// 		}
		
// 		Fly.move_voltage(flyPower);

// 		prevFlyError = flyError;


// 		printf("%.2f\n", currentSpeed);
// 		pros::lcd::print(5, "Fly: %f\n", currentSpeed);
// 		pros::lcd::print(6, "power: %f\n", flyPower);
// 		pros::delay(10);
// 	}
// }

//--------------------------// Filter //--------------------------//	
std::queue<double> smaData;

//Number of elements to average and the running sum
int window = 3;
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