#include "waypoint.h"


Waypoint::Waypoint(float xCoord, float yCoord) {
    setWaypoint(xCoord, yCoord);
}

void Waypoint::setWaypoint(float xCoord, float yCoord) {
    x = xCoord;
    y = yCoord;
}

Waypoint scalarMult(Waypoint P, float s) {
    Waypoint Ps = Waypoint(P.getX()*s, P.getY()*s);
    return Ps;
}

int sign(float num) {
    if(num > 0.0) {
        return 1;
    }
    else if(num < 0.0) {
        return -1;
    }
    else {
        return 0;
    }
}

void debug(Waypoint p) {
    std::cout << p.getX() << "\t" << p.getY() << "\n";
}

float distance(Waypoint A, Waypoint B) {
    Waypoint originVec = Waypoint(B.getX()-A.getX(), B.getY()-A.getY());
    return getLength(originVec);
}

Waypoint normalizeVect(Waypoint P) {
    float len = getLength(P);
    Waypoint U = Waypoint(P.getX()/len, P.getY()/len);
    return U;
}

float getLength(Waypoint P) {
    float pointX = P.getX();
    float pointY = P.getY();
    if(!((pointX == 0) && (pointY == 0))) {
        return std::sqrt(pow(pointX, 2) + pow(pointY, 2));
    }
    else {
        return 0;
    }
}





