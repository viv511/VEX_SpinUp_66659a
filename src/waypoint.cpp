#include "waypoint.h"

Waypoint Waypoint::scalarMult(Waypoint P, float s) {
    Waypoint Ps = Waypoint(P.getX()*s, P.getY()*s);
    return Ps;
}

float Waypoint::distance(Waypoint A, Waypoint B) {
    Waypoint originVec = Waypoint(B.getX()-A.getX(), B.getY()-A.getY());
    return getLength(originVec);
}

float Waypoint::angle(Waypoint A, Waypoint B) {
    return std::atan2(B.getY() - A.getY(), B.getX() - A.getX());
}

Waypoint Waypoint::normalizeVect(Waypoint P) {
    float len = getLength(P);
    Waypoint U = Waypoint(P.getX()/len, P.getY()/len);
    return U;
}

Waypoint Waypoint::getDirVector(Waypoint A, Waypoint B) {
    return Waypoint(B.getX()-A.getX(), B.getY()-A.getY());
}

float Waypoint::dotProduct(Waypoint A, Waypoint B) {
    float Dot = A.getX() * B.getX() + A.getY() * B.getY();
    return Dot;
}

float Waypoint::getLength(Waypoint P) {
    float pointX = P.getX();
    float pointY = P.getY();
    if(!((pointX == 0) && (pointY == 0))) {
        return std::sqrt(pow(pointX, 2) + pow(pointY, 2));
    }
    else {
        return 0;
    }
}

int Waypoint::sign(float num) {
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

void Waypoint::debug(Waypoint p) {
    std::cout << p.getX() << "\t" << p.getY() << "\t" << p.getTheta() << "\n";
}
