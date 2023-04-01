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

    public:
        PID() {
            reeeesetPlease();
            resetConstants();
        }

        PID(float kP, float kI, float kD, bool isTurnPID) {
            reeeesetPlease();
            setConstants(kP, kI, kD);
            setType(isTurnPID);
        }

        PID(float kP, float kD, bool isTurnPID) {
            reeeesetPlease();
            setConstants(kP, 0, kD);
            setType(isTurnPID);
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
        }

        void resetConstants() {
            this->constants[0] = 0;
            this->constants[1] = 0;
            this->constants[2] = 0;
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
        }

        void setType(bool turn) {
            this->isTurn = turn;
        }

    float calculateOutput(float current);

};



#endif