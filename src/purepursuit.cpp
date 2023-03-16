#include "globals.h"
#include <vector>
#include "odom.h"
#include "pros/motors.h"
#include "variables.h"
#include "driveauto.h"
#include "fly.h"
#include "cmath"
#include <iostream>
#include <fstream>
#include <filesystem>
#include "main.h"

#include "purepursuit.h"

std::vector<Waypoint> path = {{1, 1}, {100, 100}, {300, 50}, {500, 200}};

void pathFollowNormal(std::vector<Waypoint> pathToFollow) {
}

void pathFollowPurePursuit(std::vector<Waypoint> pathToFollow) {
    //Following DAWGMA Document

    //Step 1. Injecting extra points
    int inchSpacing = 6;
    std::vector<Waypoint> newPath;

    for(int lineSeg = 0; lineSeg < pathToFollow.size()-1; lineSeg++) {
        Waypoint dirVector = {pathToFollow[lineSeg+1].getX()-pathToFollow[lineSeg].getX(), pathToFollow[lineSeg+1].getY()-pathToFollow[lineSeg].getY()};
        int totalPointsFit = (int) (getLength(dirVector) / inchSpacing);
        dirVector = scalarMult(normalizeVect(dirVector), inchSpacing);

        //Inject points
        for(int i=0; i<totalPointsFit; i++) {
            newPath.push_back(Waypoint(pathToFollow[lineSeg].getX() + scalarMult(dirVector, i).getX(), pathToFollow[lineSeg].getY() + scalarMult(dirVector, i).getY()));
        }
    }

    newPath.push_back(Waypoint(pathToFollow[pathToFollow.size()-1].getX(), pathToFollow[pathToFollow.size()-1].getY()));
    //Example (a, b, tolerance) = ((1-0.7), 0.3, 0.001) use smoother function
}

std::vector<Waypoint> smooth(std::vector<Waypoint> roughPath, float a, float b, float tolerance) {
    //a is equal to "weight data"
    //b is equal to "weight smooth"
    //larger b == smoother path
    int len = roughPath.size();
    std::vector<Waypoint> smoothPath;
    for(int index = 0; index < len; index++) {
        smoothPath.push_back(roughPath[index]);
    }
    
    double change = tolerance;
	while(change >= tolerance) {
		change = 0.0;
		for(int i=1; i<len-1; i++) {
            double tempX = smoothPath[i].getX();
            smoothPath[i].setX(smoothPath[i].getX() + (a * (roughPath[i].getX() - smoothPath[i].getX()) + b * (smoothPath[i-1].getX() + smoothPath[i+1].getX() - (2.0 * smoothPath[i].getX()))));
            change += abs(tempX - smoothPath[i].getX());

            double tempY = smoothPath[i].getY();
            smoothPath[i].setY(smoothPath[i].getY() + (a * (roughPath[i].getY() - smoothPath[i].getY()) + b * (smoothPath[i-1].getY() + smoothPath[i+1].getY() - (2.0 * smoothPath[i].getY()))));
            change += abs(tempY - smoothPath[i].getY());
		}
	}

    return smoothPath;
    //Credit ~Team 2168 FRC/FTC for smoothing algorithm
}