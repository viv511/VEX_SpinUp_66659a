#include "pros/motors.h"
#include "driveauto.h"
#include <iostream>
#include <fstream>
#include <filesystem>
#include "pid.h"
#include "odom.h"
#include "fly.h"

float lin_kP = 850;
float lin_kD = 100;
float theta_kP = 400;

PID linMovement = PID(lin_kP, lin_kD, false, 1, 100, 3, 500, 3000);
PID angTurn = PID(200, 0, true, 1, 100, 3, 500, 3000);

void shoot(float rpm) {
    setFlywheelRPM(rpm);
    int counter = 0;
    while(!getReadyState()) {
        counter += 10;
        pros::delay(10);

        if(counter > 3000) {
            break;
        }
    }
    
    IIR.move_voltage(-12000);
    pros::delay(120);
    IIR.move_voltage(0);

    pros::delay(100);

    IIR.move_voltage(-12000);
    pros::delay(120);
    IIR.move_voltage(0);

    pros::delay(200);

    IIR.move_voltage(-12000);
    pros::delay(300);
    IIR.move_voltage(0);

}

void move(float dist, float limit) {
    resetX();
    linMovement.setTarget(dist);

    linMovement.setConstants(lin_kP, 0, lin_kD*fabs(dist));

    float initTheta = inertial.get_rotation();
	while(!linMovement.isSettled()) {
        float angPow = theta_kP * (inertial.get_rotation() - initTheta);
		float power = lim(linMovement.calculateOutput(getRobotPose().getX()), limit);		
        
        LeftDT.move_voltage(power - angPow);
        RightDT.move_voltage(power + angPow);
		pros::delay(10);
	}

    brake();
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
    if(fabs(theta) >= 180){
        if(theta > 0){
            theta = -1 * (360 - theta);
        }
        else{
            theta = 360 + theta;
        }
    }

    turn(theta);
}

void turn(float angle) {
    angTurn.setTarget(inertial.get_rotation() + angle);
    float fang = fabs(angle);

    //Gain Scheduling
    if(fang <= 10) {
        angTurn.setConstants(450, 0, 50);
    }
    else if(fang <= 20) {
        angTurn.setConstants(250, 0, 100);
    }
    else if(fang <= 30) {
        angTurn.setConstants(255, 0, fang*31);
    }
    else if(fang <= 40) {
        angTurn.setConstants(277, 0, fang*31);
    }
    else if(fang <= 50) {
        angTurn.setConstants(350, 0, fang*40);
    }
    else if(fang <= 60) {
        angTurn.setConstants(400, 0, fang*43);
    }
    else if(fang <= 70) {
        angTurn.setConstants(425, 0, fang*44);
    }
    else if(fang <= 80) {
        angTurn.setConstants(470, 0, fang*45);
    }
    else if(fang <= 90) {
        angTurn.setConstants(550, 0, fang*40);
    }
    else if(fang <= 105) {
        angTurn.setConstants(560, 0, fang*26);
    }
    else if(fang <= 120) {
        angTurn.setConstants(570, 0, fang*25);
    }
    else if(fang <= 135) {
        angTurn.setConstants(570, 0, fang*19);
    }
    else if(fang <= 150) {
        angTurn.setConstants(450, 0, fang*20);
    }
    else if(fang <= 165) {
        angTurn.setConstants(450, 0, fang*20);
    }
    else {
       angTurn.setConstants(500, 0, fang*25);
    }
	
	while(!angTurn.isSettled()) {
        float turnSpeed = angTurn.calculateOutput(inertial.get_rotation());
        LeftDT.move_voltage(turnSpeed);
        RightDT.move_voltage(-turnSpeed);
        
        // pros::lcd::print(3, "target: %f\n", angTurn.getTarget());
        pros::delay(10);
	}

    brake();
}

void swing(float angle, bool leftSwing, float otherPower) {
    angTurn.setTarget(inertial.get_rotation() + angle);
    float fang = fabs(angle);

    //Gain Scheduling
    if(fang <= 10) {
        angTurn.setConstants(450, 0, 50);
    }
    else if(fang <= 20) {
        angTurn.setConstants(250, 0, 100);
    }
    else if(fang <= 30) {
        angTurn.setConstants(255, 0, fang*31);
    }
    else if(fang <= 40) {
        angTurn.setConstants(277, 0, fang*31);
    }
    else if(fang <= 50) {
        angTurn.setConstants(350, 0, fang*40);
    }
    else if(fang <= 60) {
        angTurn.setConstants(400, 0, fang*43);
    }
    else if(fang <= 70) {
        angTurn.setConstants(425, 0, fang*44);
    }
    else if(fang <= 80) {
        angTurn.setConstants(470, 0, fang*45);
    }
    else if(fang <= 90) {
        angTurn.setConstants(550, 0, fang*43);
    }
    else if(fang <= 105) {
        angTurn.setConstants(550, 0, fang*44);
    }
    else if(fang <= 120) {
        angTurn.setConstants(570, 0, fang*25);
    }
    else if(fang <= 135) {
        angTurn.setConstants(570, 0, fang*20);
    }
    else if(fang <= 150) {
        angTurn.setConstants(450, 0, fang*20);
    }
    else if(fang <= 165) {
        angTurn.setConstants(450, 0, fang*20);
    }
    else {
       angTurn.setConstants(500, 0, fang*25);
    }

    while(!angTurn.isSettled()) {
        float turnSpeed = angTurn.calculateOutput(inertial.get_rotation());
        if(leftSwing){
            LeftDT.move_voltage(turnSpeed);
            RightDT.move_voltage(otherPower);
        }
        else {
            LeftDT.move_voltage(otherPower);
            RightDT.move_voltage(-turnSpeed);
        }
        
        // pros::lcd::print(3, "target: %f\n", angTurn.getTarget());
        pros::delay(10);
	}

    brake();
}

void brake() {
    LeftDT.brake();
    RightDT.brake();
}

void setHold() {
    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
}

void setCoast() {
    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

void powerMV(float l, float r) {
    LeftDT.move_voltage(l);
    RightDT.move_voltage(r);
}

float lim(float input, float limitPCT) {
    if(abs(input) >= (12000 * limitPCT)) {
        if(numbersign(input) == 1) {
            input = 12000 * limitPCT;
        }else {
            input = -12000 * limitPCT;
        }
    }

    return input;
}