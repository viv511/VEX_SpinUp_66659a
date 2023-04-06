#include "pid.h"
#include "waypoint.h"

constexpr float INTEGRAL_TURN_THRESHOLD = 3; 
constexpr float INTEGRAL_DRIVE_THRESHOLD = -1;
const int PID_DELAY_TIME = 10; 

float PID::calculateOutput(float current) {
    error = target - current;
    derivative = error - previousError;

    if(constants[1] == 0) {
        integral = 0;
    }
    else {
        if(isTurn) {
            if(fabs(error) < INTEGRAL_TURN_THRESHOLD) {
                integral = integral + error;
            }
        }
        else {
            if(fabs(error) < INTEGRAL_DRIVE_THRESHOLD) {
                integral = integral + error;
            }
        }

        if(Waypoint::sign(error) != Waypoint::sign(previousError)) {
            integral = 0;
        }
    }

    output = constants[0]*error + constants[1]*integral + constants[2]*derivative;
    previousError = error;

    return output;
}

bool PID::isSettled() {
    if(fabs(error) < smallErr) {
        smallTimeCounter += PID_DELAY_TIME;
        if(smallTimeCounter > smallExit) {
            //In target small threshold for smallExit amount of time
            return true;
        }
    }
    else if(fabs(error) < largeErr) {
        largeTimeCounter += PID_DELAY_TIME;
        if(largeTimeCounter > largeExit) {
            //In target large threshold for largeExit amount of time
            return true;
        }
    }

    //add to maxtime counter
    maxCounter += PID_DELAY_TIME;
    if(maxCounter > maxTime) {
        //took too long
        return true;
    }

    //else it hasnt reached yet
    return false;
}