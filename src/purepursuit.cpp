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
        Waypoint dirVector = Waypoint(pathToFollow[lineSeg+1].getX()-pathToFollow[lineSeg].getX(), pathToFollow[lineSeg+1].getY()-pathToFollow[lineSeg].getY());
        int totalPointsFit = (int) (getLength(dirVector) / inchSpacing);
        dirVector = scalarMult(normalizeVect(dirVector), inchSpacing);

        //Inject points
        for(int i=0; i<totalPointsFit; i++) {
            newPath.push_back(Waypoint(pathToFollow[lineSeg].getX() + scalarMult(dirVector, i).getX(), pathToFollow[lineSeg].getY() + scalarMult(dirVector, i).getY()));
        }
    }

    newPath.push_back(Waypoint(pathToFollow[pathToFollow.size()-1].getX(), pathToFollow[pathToFollow.size()-1].getY()));

    //Step 2. Smooth Path
    newPath = smooth(newPath, 0.3, 0.7, 0.001);

    //Step 3. Distance between points
    for(int i=1; i<newPath.size(); i++) {
        //Runing Sum (D_i = D_i-1 + dist(D_i, D_i-1)
        newPath[i].setDist(newPath[i-1].getDist() + distance(newPath[i], newPath[i-1]));
    }

    //Step 4. Calculate curvature (1/radius) between points
    newPath[0].setCurv(0);
    for(int i=1; i<(newPath.size()-1); i++) {
        float x1 = newPath[i-1].getX();
        float y1 = newPath[i-1].getY();
        float x2 = newPath[i].getX();
        float y2 = newPath[i].getY();
        float x3 = newPath[i+1].getX();
        float y3 = newPath[i+1].getY();

        if(x1 == y1) {
            //Account for divide by 0 edge cases
            newPath[i-1].setX(x1 + 0.001);
        }

        float kOne = 0.5 * (pow(x1, 2) + pow(y1, 2) - pow(x2, 2) - pow(y2, 2)) / (x1 - x2);
        float kTwo = (y1 - y2) / (x1 - x2);

        float b = 0.5 * (pow(x2, 2) - 2 * x2 * kOne + pow(y2, 2) - pow(x3, 2) + 2 * x3 * kOne - pow(y3, 2)) / (x3 * kTwo - y3 + y2 - x2 * kTwo);
        float a = kOne - kTwo * b;

        float r = std::sqrt(pow((x1 - a), 2) + pow((y1 - b), 2));
        float c = 1/r;

        if(std::isnan(c)) {
            //Straight line
            newPath[i].setCurv(0);
        }
        else {
            newPath[i].setCurv(c);
        }
    }
    newPath[newPath.size()-1].setCurv(0);


}

std::vector<Waypoint> smooth(std::vector<Waypoint> roughPath, float a, float b, float tolerance) {
    //a == weight on the data, b == weight for smoothing"
    //A larger "b" value will result in a smoother path, be careful!
    std::vector<Waypoint> smoothPath;
    for(int index = 0; index < roughPath.size(); index++) {
        smoothPath.push_back(roughPath[index]);
    }
    
    double change = tolerance;
	while(change >= tolerance) {
		change = 0.0;
		for(int i=1; i<roughPath.size()-1; i++) {
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