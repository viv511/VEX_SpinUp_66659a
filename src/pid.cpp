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