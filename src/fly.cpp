#include "main.h"
#include "cmath"
#include "globals.h"
#include "variables.h"
#include <queue>
#include "fly.h"

pros::Mutex rpmMutex;
double target_rpm = 0;
double threshold = 10;

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
void flySpeed() {
	bool flyState = false;
	bool flyLast = false;
	double currentSpeed = 0;

	Fly.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	while(true) {
		if((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && !flyLast) {
			flyState = !flyState;
			flyLast = true;
		}
		else if(!((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)))) {
			flyLast = false;
		}

		float target_speed = getFlywheelRPM();
		float goalRPM = target_speed/600 * 3000;
		float holdPower = goalRPM * 12000/3000;
	
		currentSpeed = SMA_Filter(6 * Fly.get_actual_velocity());

		if(target_speed == 0) {
			Fly.move_voltage(0);
		}
		else if(fabs(currentSpeed - target_speed) < threshold) {
			Fly.move_voltage(holdPower);
			controller.rumble(".");
		}
		else {
			if(currentSpeed < target_speed) {
				Fly.move_voltage(12000);
			}
			else{
				Fly.move_voltage(0);
			}
		}
		
		pros::lcd::print(6, "Fly: %f\n", currentSpeed);
		pros::delay(20);


		if(flyState == true) {
			if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){ 
				setFlywheelRPM(1800);
			}
			else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
				setFlywheelRPM(2500);
			}
			else {
				setFlywheelRPM(2200);
			}
		}
		else {
			setFlywheelRPM(0);
		}
	}
}

//--------------------------// Filter //--------------------------//	
std::queue<double> smaData;
//Number of elements to average
int window = 15;
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