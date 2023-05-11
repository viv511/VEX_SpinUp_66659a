#include "odom.h"
#include "utilities.h"
#include "pid.h"

//Using a waypoint object to hold robot's odom calculations
Waypoint robotPose = Waypoint(0, 0, 0);

//--------------------------// Odometry //--------------------------//
constexpr float PI = 3.141592;
constexpr float degreesToRadians = PI/180;
constexpr float radiansToDegrees = 180/PI;

//Distance from center
constexpr float R_DIST = 0.75;
constexpr float B_DIST = 5.6;

//Diameter of wheels
constexpr float R_DIAMETER = 2.83;
constexpr float B_DIAMETER = 2.83;

//Ticks per rotation
constexpr float R_TICKS = 36000.0;
constexpr float B_TICKS = 36000.0;

//L + Ratio (jk)
constexpr float R_RATIO = (PI * R_DIAMETER)/R_TICKS; 
constexpr float B_RATIO = (PI * B_DIAMETER)/B_TICKS;

float rightCurrent = 0;
float backCurrent = 0;
float rightLast = 0;
float backLast = 0;
float rightChange = 0;
float backChange = 0;

float absoluteRight = 0;
float absoluteBack = 0;

float curTheta = 0;
float prevTheta = curTheta;
float deltaTheta = 0.0001;

float x_local = 0;
float y_local = 0;
float offset = 0;


float x_global = 0;
float y_global = 0;

void odometry() {
	// backEncoder.reset_position();
	rightEncoder.reset_position();
	rightEncoder.set_position(0);
	
	while(true) {
		pros::lcd::print(0, "Tracking Wheel: %f\n", robotPose.getX());
		pros::lcd::print(1, "Angle: %f\n", robotPose.getTheta());

		rightCurrent = rightEncoder.get_position();
		// backCurrent = backEncoder.get_position();	
		
		//Calculate the change in encoder since last cycle, Ticks --> Inches
		rightChange = (rightCurrent - rightLast) * R_RATIO;
		// backChange = (backCurrent - backLast) * B_RATIO;

		// //Wrap theta
		// if(inertial.get_rotation() > 180) {
		// 	inertial.set_rotation(inertial.get_rotation() - 360);
		// }
		// if(inertial.get_rotation() <= -180) {
		// 	inertial.set_rotation(inertial.get_rotation() + 360);
		// }

		// //Find the angle change from the last cycle (Radians)
		// curTheta = inertial.get_rotation() * degreesToRadians;
		// deltaTheta = curTheta - prevTheta;

		//Store last values
		rightLast = rightCurrent;
		// backLast = backCurrent;
		// prevTheta = curTheta;

		// Calculate absolute orientation + absolute left, right, and back encoder change
		absoluteRight += rightChange;
		// absoluteBack += backChange;	
		
		// //Get local coords
		// if(fabs(deltaTheta) == 0) {
		// 	x_local = rightChange;
		// 	y_local = backChange;
		// }
		// else {
		// 	x_local = ((rightChange / deltaTheta) - R_DIST) * 2 * sin(deltaTheta / 2.0);
		// 	y_local = ((backChange / deltaTheta) + B_DIST) * 2 * sin(deltaTheta / 2.0);
		// }

		// //Translate to global
		// offset = inertial.get_rotation() + (deltaTheta / 2.0);

		// float a = (cos(offset) * y_local - sin(offset) * x_local);
		// float b = (cos(offset) * x_local - sin(offset) * y_local);
		// x_global += a;
		// y_global += b;

		// robotPose.setX(x_global);
		robotPose.setX(absoluteRight);
		// robotPose.setY(y_global);
		robotPose.setTheta(inertial.get_rotation());
		
		pros::delay(10);
	}
}

void resetX() {
	absoluteRight = 0;
}

float kP_Linear = 2;
float kP_Angular = 25;

void moveTo(Waypoint P) {
	LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    
	Waypoint R = getRobotPose();
	float dist = distance(R, P);
	float ang = angle(R, P);

	while(!robotSettled(P)) {
		R = getRobotPose();
		dist = distance(R, P);
		ang = angle(R, P);

		float headingScale = std::cos(ang);
		//lin power * headingScale +/- heading output 



		// xDiff = P.getX() - R.getX();
		// yDiff = P.getY() - R.getY();

		// float phi = R.getTheta() * degreesToRadians;
		// float linErr = yDiff * std::cos(phi) + xDiff * std::sin(phi);
		// float angErr = angle(R, P);

		// float linPower = kP_Linear * linErr;
		// float angPower = kP_Angular * angErr;

		// float leftPower = linPower + angPower;
		// float rightPower = linPower - angPower;

		// //Secret sauce
		// float maxP = std::max(leftPower, rightPower);
		// float minP = std::abs(std::min(leftPower, rightPower));

		// float frMax = std::max(maxP, minP);
		// float scaleMoment = (frMax > 100) ? 100 / frMax : 1;

		// controller.rumble(".");
		// // pros::lcd::print(4, "Left: %f\n", (float)(70 * scaleMoment * leftPower));
		// // pros::lcd::print(5, "Right: %f\n", (float)(70 * scaleMoment * rightPower));
		// LeftDT.move_voltage(scaleMoment * leftPower * 80);
		// RightDT.move_voltage(scaleMoment * rightPower * 80);
	}

	LeftDT.brake();
    RightDT.brake();
    LeftDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
    RightDT.set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
}

void setRobotPose(Waypoint newRobotPose) {
	robotPose.setX(newRobotPose.getX());
	robotPose.setY(newRobotPose.getY());
	robotPose.setTheta(newRobotPose.getTheta());
	// robotPose = newRobotPose;
}

Waypoint getRobotPose() {
	return robotPose;
}

Waypoint scalarMult(Waypoint P, float s) {
	Waypoint Ps = Waypoint(P.getX()*s, P.getY()*s);
	return Ps;
}

float distance(Waypoint A, Waypoint B) {
	Waypoint originVec = Waypoint(B.getX()-A.getX(), B.getY()-A.getY());
	return getLength(originVec);
}

float angle(Waypoint A, Waypoint B) {
	float dTheta = std::atan2(B.getY() - A.getY(), B.getX() - A.getX());

	return dTheta*radiansToDegrees;
}

Waypoint normalizeVect(Waypoint P) {
	float len = getLength(P);
	Waypoint U = Waypoint(P.getX()/len, P.getY()/len);
	return U;
}

Waypoint getDirVector(Waypoint A, Waypoint B) {
	return Waypoint(B.getX()-A.getX(), B.getY()-A.getY());
}

float dotProduct(Waypoint A, Waypoint B) {
	float Dot = A.getX() * B.getX() + A.getY() * B.getY();
	return Dot;
}

float getLength(Waypoint P) {
	float pointX = P.getX();
	float pointY = P.getY();
	if(!((pointX == 0) && (pointY == 0))) {
		return std::sqrt(pow(pointX, 2) + pow(pointY, 2));
	}
	else {
		return 0;
	}
}

int numbersign(float num) {
	if(num > 0.0) {
		return 1;
	}
	else if(num < 0.0) {
		return -1;
	}
	else {
		return 0;
	}
}

void debug(Waypoint p) {
	std::cout << p.getX() << "\t" << p.getY() << "\t" << p.getTheta() << "\n";
}

bool robotSettled(Waypoint A) {
	Waypoint current = getRobotPose();
	if(fabs(A.getX()-current.getX()) < 2) {
		if(fabs(A.getY()-current.getY()) < 2) {
			if(fabs(A.getTheta()-current.getTheta()) < 2) {
				return true;
			}
		}
	}

	return false;
}