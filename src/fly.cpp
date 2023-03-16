#include "main.h"
#include "cmath"
#include "globals.h"
#include "variables.h"
#include <queue>
#include "fly.h"

//--------------------------// Mutex Management //--------------------------//	
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
int rpmThreshold = 100;
bool flyLast = false;

void flySpeed() {
	float kV = 3.325;
	float kS = 975;
	
	float flyPower = 0;

	//CRUCIAL!!
	Fly.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	while(true) {
		if(autonThreshold) {
			rpmThreshold = 20;
		}
		else {
			rpmThreshold = 100;
		}

		//Toggle Logic
		if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && !flyLast) {
			flyState = !flyState;
			flyLast = true;
		}
		else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))) {
			flyLast = false;
		}

		//Note: Multiply by 6 because direct cartridge is 3600 RPM
		float targetSpeed = getFlywheelRPM();
		kV = 3.32 + 0.005 * (targetSpeed-2200);
		float currentSpeed = SMA_Filter(6 * Fly.get_actual_velocity());

		//Calculate error
		flyError = targetSpeed - currentSpeed;

		//Set speed
		if(targetSpeed == 0) {
			flyPower = 0;
		}
		else {
			//Bang-Bang
			if(flyError > rpmThreshold){
				flyPower = 12000;
			}
			else if(flyError < -rpmThreshold) {
				flyPower = 0;
			}
			else {	
				//Feedforward in y=mx+b form
				flyPower = (kV * targetSpeed) + kS;
			}

		}
		
		//Voltage Caps
		if(flyPower < 0) {
			flyPower = 0;
		}
		if(flyPower > 12000) {
			flyPower = 12000;
		}

		Fly.move_voltage(flyPower);

		//Debugging Utils
		pros::lcd::print(5, "Fly: %f\n", currentSpeed);
		pros::lcd::print(6, "power: %f\n", flyPower);
		pros::delay(10);
	}
}

//--------------------------// Filter //--------------------------//	
std::queue<double> smaData;

//Number of elements to average and the running sum
int window = 5;
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