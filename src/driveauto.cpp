#include "main.h"
#include "cmath"
#include "globals.h"
#include "odom.h"
#include "pros/motors.h"
#include "variables.h"
#include "driveauto.h"

const double inertialDrift = 1.00446429;

void driveOdomAngPD(int inches, double limit, double f_kP, double f_kD, double f_kP_Theta) {
    inertial.tare_rotation();
    absoluteRight = 0;
    float curInches = absoluteRight;
    float goal = curInches + inches;

    //1 or -1, forward or backward
    int dir = abs(inches)/inches;

    //save initial heading for angle correction
    float initHeading = inertial.get_rotation();

    float derivative = 0;
    float prevLatError = 0;
    int latPower = 0;

    int angPower = 0;
    bool turnRight;

    int timeAtError = 0;

    //Constants
    float kP = f_kP;
    float kD = f_kD;
    float kP_Theta = f_kP_Theta;

    do {
        curInches = absoluteRight;

        //Calculate error from desired
        float latError = fabs(inches - curInches);
        float angError = initHeading - inertial.get_rotation();

        controller.rumble(".");

        //Derivative term on lateral
        derivative = latError - prevLatError;
        prevLatError = latError;
        
        //Check which direction to turn
        if(angError > 0) {
            turnRight = false;
        }
        else {
            turnRight = true;
        }

        //Calculate lateral power (Proportional + Derivative)
        latPower = (kP * latError) + (kD * derivative);

        //Use limit to cap the motor's max output voltage (lateral)
        if(latPower >= (12000 * limit)) {
			latPower = 12000 * limit;
		}

        if(fabs(angError) < 1) {
            angPower = 0;
        }
        else {
            if(turnRight) {
                //Turn right
                angPower = angError * kP_Theta;
            }
            else {
                //Turn left
                angPower = angError * kP_Theta * -1;
            }

            //Apply same limit to angle
            if(angPower >= 3000) {
                angPower = 3000;
            }
        }

        if(fabs(latError) < 2) {
            angPower = 0;
        }

        if(fabs(latError) < 3) {
            timeAtError+=10;
        }
        

        if(timeAtError > 1000) {
            break;
        }

        latPower *= dir;

        LeftDT.move_voltage(latPower + angPower);
        RightDT.move_voltage(latPower - angPower);


        pros::delay(10);
    }while(fabs(goal - curInches) > 1);

    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    LeftDT.brake();
    RightDT.brake();
    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

void oldDriveArcPD(int leftTicks, int rightTicks, double limit, int dir) {
    //Reset motor ticks
    reset_encoder();

    //convert inches --> ticks
    int leftD = 0;
    int rightD = 0;
    int leftError = 0;
    int rightError = 0;
    int prevLeftError = 0;
    int prevRightError = 0;
    
    int leftPower = 0;
    int rightPower = 0;

    //Constants
    float kP = 25;
    float kD = 0;

    do {
        //Calculate error from desired
        leftError = leftTicks - avg_l();
        rightError = rightTicks - avg_r();

        //Derivative term on lateral
        leftD = leftError - prevLeftError;
        rightD = rightError - prevRightError;

        prevLeftError = leftError;
        prevRightError = rightError;
       
        //Calculate lateral power (Proportional + Derivative)
        leftPower = (kP * leftError) + (kD * leftD);
        rightPower = (kP * rightError) + (kD * rightD);

        //Use limit to cap the motor's max output voltage (lateral)
        if(leftPower >= (12000 * limit)) {
			leftPower = 12000 * limit;
		}
        if(rightPower >= (12000 * limit)) {
			rightPower = 12000 * limit;
		}

        LeftDT.move_voltage(leftPower * dir);
        RightDT.move_voltage(2 * rightPower * dir);

        pros::delay(10);
    }while((avg_l() < leftTicks) && (avg_r() < rightTicks));
}


void turn(double angle) {
    int timeAtError = 0;
    int totalTime = 0;
	// inertial.set_rotation(0.0);

	double error = 0;
	double prevError = 0;
	double derivative = 0;
	double power = 0;
	double kP = 130;
	double kD = 180;
    double kI = 10;
    double integral = 0;
    //315, 270, 10

	double target = angle + inertial.get_rotation();
	
	do {
		error = target - inertial.get_rotation();
		derivative = error - prevError;
		prevError = error;

        if(std::signbit(error) != std::signbit(prevError)) {
            integral = 0;
        }
        if(error > 15) {
            integral = 0;
        }

		power = (error * kP) + (derivative*kD) + (integral * kI);

		RightDT.move_voltage(-power);
		LeftDT.move_voltage(power);

        pros::lcd::print(3, "theta: %f\n", inertial.get_rotation());
        pros::lcd::print(4, "power: %f\n", power);
        pros::lcd::print(5, "error: %f\n", error);

        if(fabs(target) > 100) {
           if(fabs(target - inertial.get_rotation()) < 5) {
                timeAtError+=10;
            } 
        }
        else if(fabs(target) > 15) {
            if(fabs(target - inertial.get_rotation()) < 3) {
                timeAtError+=10;
            }
        }
        else {
            if(fabs(target - inertial.get_rotation()) < 1) {
                timeAtError+=10;
            }
        }

        totalTime+=10;

        if(timeAtError > 750) {
            controller.rumble("-");
            break;
        }

        if(totalTime > 2000) {
            controller.rumble("-");
            break;
        }

        pros::delay(10);
        // std::cout << error << "," << power << "\n";

	}while(fabs(target - inertial.get_rotation()) > 0.5);

    // if(fabs(target - inertial.get_rotation()) > 10) {
    //     turn(target - inertial.get_rotation());
    // }

    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    LeftDT.brake();
    RightDT.brake();
    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

void rotate(double angle) {
    int totalTime = 0;
	double error = 0;
	double prevError = 0;
	double derivative = 0;
	double power = 0;

    double kP = 130;
    double kI = 8;
    double integral = 0;
    double kD = 170;

	double target = angle + inertial.get_rotation();
	
	do {
		error = target - inertial.get_rotation();
		derivative = error - prevError;
        integral = integral + error;
        totalTime+=10;



		RightDT.move_voltage(-power);
		LeftDT.move_voltage(power);

        pros::lcd::print(3, "theta: %f\n", inertial.get_rotation());
        pros::lcd::print(4, "power: %f\n", power);
        pros::lcd::print(5, "error: %f\n", error);

        pros::delay(15);
        prevError = error;

        if(totalTime > 2000) {
            controller.rumble("-");
            break;
        }
	}while(fabs(target - inertial.get_rotation()) > 0.5);

    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    LeftDT.brake();
    RightDT.brake();
    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}