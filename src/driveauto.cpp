#include "main.h"
#include "cmath"
#include "globals.h"
#include "odom.h"
#include "variables.h"
#include "driveauto.h"

int angularCorrectionLimit = 2500;

//Where 0 <= limit <= 1, as a percentage
void driveAngPD(int ticks, double limit) {
    //Reset motor ticks
    reset_encoder();

    //1 or -1, forward or backward
    int dir = abs(ticks)/ticks;

    //save initial heading for angle correction
    float initHeading = inertial.get_rotation();

    //convert inches --> ticks
    float derivative = 0;
    float prevLatError = 0;
    int latPower = 0;

    int angPower = 0;
    bool turnRight;

    //Constants
    float kP = 1;
    float kD = 1;
    float kP_Theta = 1;
    do {
        //Calculate error from desired
        float latError = abs(ticks) - average_encoders();
        float angError = initHeading - inertial.get_rotation();

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

        if(turnRight) {
            //Turn right
            angPower = angError * kP_Theta;
        }
        else {
            //Turn left
            angPower = angError * kP_Theta * -1;
        }

        //Apply same limit to angle
        if(angPower >= angularCorrectionLimit) {
            angPower = angularCorrectionLimit;
        }

        latPower *= dir;

        LeftDT.move_voltage(latPower + angPower);
        RightDT.move_voltage(latPower - angPower);

        pros::delay(10);
    }while(average_encoders() < abs(ticks));
}

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

    //Constants
    float kP = f_kP;
    float kD = f_kD;
    float kP_Theta = f_kP_Theta;

    do {
        curInches = absoluteRight;

        //Calculate error from desired
        float latError = fabs(inches - curInches);
        float angError = initHeading - inertial.get_rotation();

        pros::lcd::print(2, "laterror: %f\n", latError);
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
            if(angPower >= angularCorrectionLimit) {
                angPower = angularCorrectionLimit;
            }
        }

        latPower *= dir;

        LeftDT.move_voltage(latPower + angPower);
        RightDT.move_voltage(latPower - angPower);


        pros::delay(10);
    }while(fabs(goal - curInches) > 1);
}