#include "main.h"
#include "cmath"
#include "globals.h"
#include "odom.h"
#include "pros/motors.h"
#include "variables.h"
#include "driveauto.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include "fly.h"

const double inertialDrift = 1.00446429;

double pdVals[4][3];

void setPIDvalues() {
    //45 Degrees
    pdVals[0][0] = 201.5;
    pdVals[0][1] = 0;
    pdVals[0][2] = 45;

    //90 Degrees
    pdVals[1][0] = 201;
    pdVals[1][1] = 10500;
    pdVals[1][2] = 90;

    //135 Degrees
    pdVals[2][0] = 146;
    pdVals[2][1] = 210;
    pdVals[2][2] = 135;

    //180
    pdVals[3][0] = 140;
    pdVals[3][1] = 220;
    pdVals[3][2] = 180;
}

void bucket(int rpmSpeed, int threshold, int timeout) {
    setFlywheelRPM(rpmSpeed);
    for(int indexNum = 0; indexNum < 3; indexNum++) {
        pros::delay(100);
        if((indexNum == 0) && (rpmSpeed > 2500)) {
            pros::delay(100);
        }

        int timeSet = pros::millis();
        while(flyError > threshold) {
            pros::delay(10);

            if(pros::millis()-timeSet > timeout) {
                controller.rumble("-");
                break;
            }
        }

        pros::delay(200);
        index(indexNum);
    }
}

void shoot(int num_disks, int rpmSpeed, int timeout, int threshold, int waitMsec) {
    flyState = true;
    setFlywheelRPM(rpmSpeed);
    int timeSet;

    for(int i=0; i<num_disks; i++) {
        pros::delay(waitMsec);
        timeSet = 0;
        while(flyError > threshold) {
            pros::delay(10);
            timeSet+=10;

            if(timeSet > timeout) {
                controller.rumble("-");
                break;
            }
        }

        index(i);
    }
}

void index(int disk) {
    IIR.move_voltage(-12000);
	controller.rumble(".");
    if(disk == 2) {
        pros::delay(200);
    }
    else if(disk == 1) {
        pros::delay(140);
    }
    else {
        pros::delay(120);
    }

	IIR.move_voltage(0);
}

void pivot(double angle) {
    //theta = total turn
    //angle = target angle (0 to 360 degrees)
    //startAngle = current angle of robot -inf < degrees < inf
    double theta = 0;
    double startAngle = inertial.get_rotation();

    // startAngle mod stuff - getting relative to (0 to 360 degrees)
    int quotient;
    if(startAngle < 0){
        quotient = -1 * int(startAngle / 360) + 1;
        startAngle = startAngle + quotient * 360;
    }
    else{
        quotient = int(startAngle / 360);
        startAngle = startAngle - quotient * 360;
    }

    // target angle fr
    theta = angle - startAngle;
    if(fabs(theta) > 180){
        if(theta > 0){
            theta = -1 * (360 - theta);
        }
        else{
            theta = 360 + theta;
        }
    }

    // if bot is skill issue with negative
    // if(theta < 0){
    //     theta = 360 + theta;
    // }

    // turn!!
    turn(theta);
	// //theta = total turn
	// //angle = target angle (0 to 360 degrees)
	// //startAngle = current angle of robot -inf < degrees < inf
	// double theta = 0;
	// int startAngle = inertial.get_rotation();
	// //get relative to (0 to 360 degrees)
	// startAngle = startAngle % 360;

	// //two cases
	// if(startAngle > 180) {
	// 	theta = 360 - startAngle;
	// 	theta += angle;
	// 	if(theta >= 180) {
	// 		//if the turn is bigger than 180, impractical - turn the other way
	// 		theta = 360 - theta;
	// 		theta*=-1;
	// 	}
	// }
	// else if(startAngle <= 180) {
	// 	theta = (theta + startAngle) * -1;
	// 	theta+=angle;
	// 	if(theta >= 180) {
	// 		//if the turn is bigger than 180, impractical - turn the other way
	// 		theta = 360 - theta;
	// 		theta*=-1;
	// 	}
	// }

	// //theta is final how much to turn
	// turn(theta);
}

void forwardPD(int inches, double limit) {
    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);

    absoluteBack = 0;
    float curInches = 0;

    //1 or -1, forward or backward
    float latError;
    int dir = abs(inches)/inches;

    //save initial heading for angle correction
    float initHeading = inertial.get_rotation();

    float derivative = 0;
    float prevLatError = 0;
    int latPower = 0;

    int smallError = 0;
    int largeError = 0;

    //Constants
    float kP = 775;
    float kD = abs(inches) * 900;
    float kP_Theta = 70;

    do {
        pros::lcd::print(2, "inertial: %f\n", inertial.get_rotation());
        // pros::lcd::print(3, "ang: %f\n", angError);
        curInches = absoluteBack;

        //Calculate error from desired
        latError = fabs(inches - curInches);

        //Derivative term on lateral
        derivative = (latError - prevLatError)/10;
        prevLatError = latError;
        
        //Calculate lateral power (Proportional + Derivative)
        latPower = dir * ((kP * latError) + (kD * derivative));

        //Use limit to cap the motor's max output voltage (lateral)
        if(abs(latPower) >= (12000 * limit)) {
			if(dir == 1) {
                latPower = 12000 * limit;
            }else if(dir == -1) {
                latPower = -12000 * limit;
            }
		}

        if(fabs(latError) <= 1) {
            smallError+=10;
        }
        if(fabs(latError) <= 3) {
            largeError+=10;
        }
        
        
        if(smallError > 100) {
            controller.rumble("-");
            break;
        }
        if(largeError > 500) {
            controller.rumble(".");
            break;
        }

        LeftDT.move_voltage(latPower + (kP_Theta * (initHeading - inertial.get_rotation())));
        RightDT.move_voltage(latPower - (kP_Theta * (initHeading - inertial.get_rotation())));

        pros::delay(10);
    }while(true);

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


void turn(float angle) {
    int smallErr = 0;
    int largeErr = 0;
    int dir = fabs(angle)/angle;

	float error = 0;
	float prevError = 0;
	float derivative = 0;
    float integral = 0;
	float power = 0;
	float kP = 190;
    if(angle < 90) {
        kP -= 15;
    }
    float kI = 6;
    float kD = fabs(angle) * 150;

	float target = angle + inertial.get_rotation();
	
	do {
		error = target - inertial.get_rotation();
		derivative = (error - prevError)/10;
		prevError = error;

        integral = integral + error;

        if(error > 10) {
            integral = 0;
        }

		power = (error * kP) + (derivative*kD) + (integral * kI);
        // power = (error * kP) + (derivative * kD);

        //Cap
        if(fabs(power) >= 12000) {
            if(dir == 1) {
                power = 12000;
            } 
            else if(dir == -1) {
                power = -12000;
            }
        }

		RightDT.move_voltage(-power);
		LeftDT.move_voltage(power);

        pros::lcd::print(3, "theta: %f\n", inertial.get_rotation());
        pros::lcd::print(4, "power: %f\n", power);

        if(fabs(error) <= 1) {
            smallErr+=10;
        }
        if(fabs(error) <= 3) {
            largeErr+=10;
        }  

        if(smallErr > 100) {
            controller.rumble(".");
            break;
        }
        if(largeErr > 500) {
            controller.rumble("-");
            break;
        }

        pros::delay(10);
	}while(true);

    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    LeftDT.brake();
    RightDT.brake();
    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

void rotate(double angle) {
    int timeAtError = 0;
    int totalTime = 0;

	double error = 0;
	double prevError = 0;
	double derivative = 0;
	double power = 0;
	double kP = 95;
	double kI = 6.5;
    double kD = 220;
    double integral = 0;
    //100, 7, 220

	double target = angle + inertial.get_rotation();
	
	do {
		error = target - inertial.get_rotation();
		derivative = error - prevError;
        integral+=error;
		prevError = error;

        if(std::signbit(error) != std::signbit(prevError)) {
            integral = 0;
        }
        if(error > 10) {
            integral = 0;
        }

		power = (error * kP) + (derivative*kD) + (integral * kI);

		RightDT.move_voltage(5000 - 0.6 * power);
		LeftDT.move_voltage(5000 + 0.6 * power);

        pros::lcd::print(3, "theta: %f\n", inertial.get_rotation());
        pros::lcd::print(4, "power: %f\n", power);

        if(fabs(target - inertial.get_rotation()) < 3) {
            timeAtError+=10;
        }
        
        totalTime+=10;

        if(timeAtError > 1000) {
            controller.rumble("-");
            break;
        }

        if(totalTime > 5000) {
            controller.rumble("-");
            break;
        }

        pros::delay(10);
        // std::cout << totalTime << "," << power << "," << error << "\n";

	}while(fabs(target - inertial.get_rotation()) > 1);

    // pros::lcd::print(5, "error: %f\n", target - inertial.get_rotation());

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

void negative(double angle, float p, float d) {
    int timeAtError = 0;
    int totalTime = 0;

	double error = 0;
	double prevError = 0;
	double derivative = 0;
	double power = 0;
    double kP = p;
	double kI = 0;
    double kD = d;
    double integral = 0;
    //100, 6.5, 220

	double target = angle + inertial.get_rotation();
	
	do {
		error = target - inertial.get_rotation();
		derivative = error - prevError;
        integral+=error;
		prevError = error;

        if(std::signbit(error) != std::signbit(prevError)) {
            integral = 0;
        }
        if(error > 10) {
            integral = 0;
        }

		power = (error * kP) + (derivative*kD) + (integral * kI);

		RightDT.move_voltage(-power);
		LeftDT.move_voltage(power);

        pros::lcd::print(3, "theta: %f\n", inertial.get_rotation());
        pros::lcd::print(4, "power: %f\n", power);

        if(fabs(target - inertial.get_rotation()) < 3) {
            timeAtError+=10;
        }
        
        totalTime+=10;

        // if(timeAtError > 3000) {
        //     controller.rumble("-");
        //     break;
        // }

        if(totalTime > 3000) {
            controller.rumble("-");
            break;
        }

        pros::delay(10);
        // std::cout << totalTime << "," << power << "," << error << "\n";

	}while(fabs(target - inertial.get_rotation()) > 1);


    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    LeftDT.brake();
    RightDT.brake();
    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}