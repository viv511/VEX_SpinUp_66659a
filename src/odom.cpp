#include "odom.h"
#include "utilities.h"

//Using a waypoint object to hold robot's odom calculations
Waypoint robotPose = Waypoint(0, 0, 0);

//--------------------------// Odometry //--------------------------//
constexpr float degreesToRadians = PI/180;
constexpr float radiansToDegrees = 180/PI;

//Distance from center
constexpr float R_DIST = 0.75;
constexpr float B_DIST = 11.6;

//Diameter of wheels
constexpr float R_DIAMETER = 2.83;
constexpr float B_DIAMETER = 2.83;

//Ticks per rotation
constexpr float R_TICKS = 36000.0;
constexpr float B_TICKS = 360.0;

//L + Ratio (jk)
constexpr float R_RATIO = (PI * R_DIAMETER)/R_TICKS; 
constexpr float B_RATIO = (PI * B_DIAMETER)/B_TICKS;

// float rightCurrent = 0;
float backCurrent = 0;
// float rightLast = 0;
float backLast = 0;
// float rightChange = 0;
float backChange = 0;

// float absoluteRight = 0;

float prevTheta = curTheta;
float deltaTheta;

// float x_local = 0;
// float y_local = 0;
// float offset = 0;

void odometry() {
	// rightEncoder.set_position(0);
	backEncoder.reset();

	while(true) {
		pros::lcd::print(1, "inches: %f\n", absoluteBack);
		
		// rightCurrent = -1 * rightEncoder.get_position();
		backCurrent = backEncoder.get_value();	
		
		//Calculate the change in encoder since last cycle, Ticks --> Inches
		// rightChange = (rightCurrent - rightLast) * R_RATIO;
		backChange = (backCurrent - backLast) * B_RATIO;

		// //Wrap theta
		// if(inertial.get_rotation() > 180) {
		// 	inertial.set_rotation(inertial.get_rotation() - 360);
		// }
		// if(inertial.get_rotation() <= -180) {
		// 	inertial.set_rotation(inertial.get_rotation() + 360);
		// }

		//Find the angle change from the last cycle (Radians)
		// curTheta = inertial.get_rotation() * degreesToRadians;
		// deltaTheta = curTheta - prevTheta;

		//Store last values
		// rightLast = rightCurrent;
		backLast = backCurrent;
		// prevTheta = curTheta;

		// Calculate absolute orientation + absolute left, right, and back encoder change
		// absoluteRight += rightChange;
		absoluteBack += backChange;	
		
		//Get local coords
		// if(fabs(deltaTheta) < 0.005) {
		// 	x_local = rightChange;
		// 	y_local = backChange;
		// }
		// else {
		// 	x_local = ((rightChange / deltaTheta) - R_DIST) * 2 * sin(deltaTheta / 2.0);
		// 	y_local = ((backChange / deltaTheta) + B_DIST) * 2 * sin(deltaTheta / 2.0);
		// }

		// //Translate to global
		// offset = curTheta + (deltaTheta / 2.0);

		// x_global += (cos(offset) * y_local - sin(offset) * x_local);
		// y_global += (cos(offset) * x_local - sin(offset) * y_local);
		robotPose.setX(x_global);
		robotPose.setY(y_global);
		robotPose.setTheta(inertial.get_rotation());
		
		pros::delay(10);
	}
}

void setRobotPose(Waypoint newRobotPose) {
	// robotPose.setX(newRobotPose.getX());
	// robotPose.setY(newRobotPose.getY());
	// robotPose.setTheta(newRobotPose.getTheta());
	robotPose = newRobotPose;
}

Waypoint getRobotPose() {
	return robotPose;
}