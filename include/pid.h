#include "main.h"

#pragma once
#ifndef PID_H
#define PID_H

using namespace pros;


class PID {
    private:
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

            this->smallTimeCounter = 0;
            this->largeTimeCounter = 0;
            this->maxTime = 0;
        }

        float getTarget() {
            return target;
        }

        void setTarget(float t) {
            this->target = t;
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

    float calculateOutput(float current);
    bool isSettled();

};



#endif