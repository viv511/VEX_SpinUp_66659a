#include "main.h"
#include "odom.h"
// #include "waypoint.h"-

// #pragma once
#ifndef PID_H
#define PID_H

using namespace pros;

constexpr float INTEGRAL_TURN_THRESHOLD = 3; 
constexpr float INTEGRAL_DRIVE_THRESHOLD = -1;
const int PID_DELAY_TIME = 10; 

class PID {
    public:
        //care mostly about target
        float target;

        float error;
        bool isTurn;
        float previousError;
        float integral;
        float derivative;
        float output;

        //kP, kI, kD
        float constants[3];

        float smallTimeCounter;
        float smallErr;
        float smallExit;
        float largeTimeCounter;
        float largeErr;
        float largeExit;

        float maxCounter;
        float maxTime;


    public:
        PID() {
            reeeesetPlease();
        }

        PID(float kP, float kI, float kD, bool isTurnPID, float sErr, float sExit, float lErr, float lExit, float maxAllowedTime) {
            reeeesetPlease();
            setConstants(kP, kI, kD);
            setType(isTurnPID);
            setExitConditions(sErr, sExit, lErr, lExit, maxAllowedTime);
        }

        PID(float kP, float kD, bool isTurnPID, float sErr, float sExit, float lErr, float lExit, float maxAllowedTime) {
            reeeesetPlease();
            setConstants(kP, 0, kD);
            setType(isTurnPID);
            setExitConditions(sErr, sExit, lErr, lExit, maxAllowedTime);
        }

        //best ref fr
        void reeeesetPlease() {
            this->target = 0;
            this->isTurn = true;
            this->error = 0;
            this->previousError = 0;
            this->output = 0;
            this->derivative = 0;
            this->integral = 0;
            resetConstants();
        }

        void resetConstants() {
            this->constants[0] = 0;
            this->constants[1] = 0;
            this->constants[2] = 0;

            this->smallTimeCounter = 0;
            this->largeTimeCounter = 0;
            this->maxCounter = 0;
        }

        void setConstants(float p, float i, float d) {
            this->constants[0] = p;
            this->constants[1] = i;
            this->constants[2] = d;
        }

        float getTarget() {
            return target;
        }

        void setTarget(float t) {
            this->target = t;
            this->smallTimeCounter = 0;
            this->largeTimeCounter = 0;
            this->maxCounter = 0;
        }

        void setType(bool turn) {
            this->isTurn = turn;
        }

        void setExitConditions(float sErr, float sExit, float lErr, float lExit, float maxOut) {
            this->smallErr = sErr;
            this->smallExit = sExit;
            this->largeErr = lErr;
            this->largeExit = lExit;
            this->maxTime = maxOut;
        }

        float calculateOutput(float current) {
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

                if(numbersign(error) != numbersign(previousError)) {
                    integral = 0;
                }
            }

            output = constants[0]*error + constants[1]*integral + constants[2]*derivative;
            previousError = error;

            updateTimers();

            return output;
        }

        void updateTimers() {
            if(fabs(error) < smallErr) {
                smallTimeCounter += PID_DELAY_TIME;
            }
            else if(fabs(error) < largeErr) {
                largeTimeCounter += PID_DELAY_TIME;
            }

            maxCounter += PID_DELAY_TIME;
        }

        bool isSettled() {
            if(smallTimeCounter > smallExit) {
                //In target small threshold for smallExit amount of time
                pros::lcd::print(6, "diff: %f\n", error);
                pros::lcd::print(5, "Exit: %s\n", "SMALL");
                return true;
            }
            else if(largeTimeCounter > largeExit) {
                //In target large threshold for largeExit amount of time
                pros::lcd::print(6, "diff: %f\n", error);
                pros::lcd::print(5, "Exit: %s\n", "LARGE");
                return true;
            }
            else if(maxCounter > maxTime) {
                //took too long
                pros::lcd::print(6, "diff: %f\n", error);
                pros::lcd::print(5, "Exit: %s\n", "MAX");
                return true; 
            }

            //else it hasnt reached yet
            return false;
        }

};



#endif