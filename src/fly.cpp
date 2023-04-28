#include "fly.h"

//--------------------------// Mutex Management //--------------------------//	
pros::Mutex rpmMutex;
double target_rpm = 0;

pros::Mutex rpmReady;
bool flywheel_ready = false;

void setFlywheelRPM(double targetRPM) {
    rpmMutex.take();
    target_rpm = targetRPM;
    rpmMutex.give();
}

double getFlywheelRPM() {
    double tempRPM;
    rpmMutex.take();
    tempRPM = target_rpm;
    rpmMutex.give();
    return tempRPM;
}

void setReadyState(bool flywheelState) {
	rpmReady.take();
	flywheel_ready = flywheelState;
	rpmReady.give();
}

bool getReadyState() {
	double tempState;
	rpmReady.take();
	tempState = flywheel_ready;
	rpmReady.give();
	return tempState;
}


//--------------------------// FlyWheel //--------------------------//	
int rpmThreshold = 20;
bool flyState = false;
bool flyLast = false;

bool getToggle() {
	return flyState;
}
void setToggle(bool e) {
	flyState = e;
}

void flySpeed() {
	float kV = 3.4;
	float kS = 600;
	
	float flyPower = 0;
	float flyError = 0;

	//CRUCIAL!!
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

		//Note: Multiply by 6 because direct cartridge is 3600 RPM
		float targetSpeed = getFlywheelRPM();
		float currentSpeed = SMA_Filter(6 * Fly.get_actual_velocity());
		// float currentSpeed = EMA_Filter(SMA_Filter(6 * Fly.get_actual_velocity()));

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
				setReadyState(false);
			}
			else if(flyError < -rpmThreshold) {
				flyPower = 0;
				setReadyState(false);
			}
			else {	
				//Feedforward in y=mx+b form
				flyPower = (kV * targetSpeed) + kS;
				setReadyState(true);
			}
		}
		
		//Voltage Caps
		if(flyPower < 0) {
			flyPower = 0;
		}
		if(flyPower > 12000) {
			flyPower = 12000;
		}
		
		if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
			flyPower = -12000;
		}
		Fly.move_voltage(flyPower);
		if((targetSpeed != 0) && (fabs(flyError) < 50)) {
			controller.rumble(".");
		}

		//Debugging Utils
		pros::lcd::print(7, "Fly: %.2f\n", currentSpeed);
		std::cout << pros::millis() << "," << currentSpeed << "\n";
		// pros::lcd::print(6, "power: %f\n", flyPower);
		pros::delay(10);
	}
}

//--------------------------// Filter //--------------------------//	
std::queue<double> smaData;

//Number of elements to average and the running sum
int window = 5;
double windowTotal = 0;

//EMA stuff
const double emaAlpha = 2/(1.0+window);
double emaLast = 0;

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

double EMA_Filter(double smaData) {
	//Store last ema value in "emaLast", update and return
	emaLast = (smaData * emaAlpha) + emaLast * (1 - emaAlpha);
	return emaLast;
}