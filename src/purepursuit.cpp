#include "purepursuit.h"

std::vector<Waypoint> path = {{1, 1}, {100, 100}, {300, 50}, {500, 200}};

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
    //Example (a, b, tolerance) = ((1-0.8), 0.8, 0.001)
}

void pathFollowNormal(std::vector<Waypoint> pathToFollow) {
    //L
}

std::vector<Waypoint> smooth(std::vector<Waypoint> roughPath, float a, float b, float tolerance) {
    //a is equal to "weight data"
    //b is equal to "weight smooth"
    //larger b == smoother path

    std::vector<Waypoint> smoothPath;
    


    return smoothPath;
    //Credit ~Team 2168 FRC/FTC for smoothing algorithm
}