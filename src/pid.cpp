#include "pid.h"
#include "waypoint.h"

constexpr float INTEGRAL_TURN_THRESHOLD = 3; 
constexpr float INTEGRAL_DRIVE_THRESHOLD = -1; 

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
        smallTimeCounter += 10;
        if(smallTimeCounter > smallExit) {
            //In target small threshold for smallExit amount of time
            return true;
        }
    }
    else if(fabs(error) < largeExit) {
        largeTimeCounter += 10;
        if(largeTimeCounter > largeExit) {
            //In target large threshold for largeExit amount of time
            return true;
        }
    }

    //add to maxtime counter
    maxCounter += 10;
    if(maxCounter > maxTime) {
        //took too long
        return true;
    }

    //else it hasnt reached yet
    return false;
}