// #include "waypoint.h"

// Waypoint scalarMult(Waypoint P, float s) {
//     Waypoint Ps = Waypoint(P.getX()*s, P.getY()*s);
//     return Ps;
// }

// float distance(Waypoint A, Waypoint B) {
//     Waypoint originVec = Waypoint(B.getX()-A.getX(), B.getY()-A.getY());
//     return getLength(originVec);
// }

// float angle(Waypoint A, Waypoint B) {
//     float dTheta = std::atan2(B.getY() - A.getY(), B.getX() - A.getX());
//     if(dTheta < 0) {
//         dTheta += 360;
//     }
//     return dTheta;
// }

// Waypoint normalizeVect(Waypoint P) {
//     float len = getLength(P);
//     Waypoint U = Waypoint(P.getX()/len, P.getY()/len);
//     return U;
// }

// Waypoint getDirVector(Waypoint A, Waypoint B) {
//     return Waypoint(B.getX()-A.getX(), B.getY()-A.getY());
// }

// float dotProduct(Waypoint A, Waypoint B) {
//     float Dot = A.getX() * B.getX() + A.getY() * B.getY();
//     return Dot;
// }

// float getLength(Waypoint P) {
//     float pointX = P.getX();
//     float pointY = P.getY();
//     if(!((pointX == 0) && (pointY == 0))) {
//         return std::sqrt(pow(pointX, 2) + pow(pointY, 2));
//     }
//     else {
//         return 0;
//     }
// }

// int sign(float num) {
//     if(num > 0.0) {
//         return 1;
//     }
//     else if(num < 0.0) {
//         return -1;
//     }
//     else {
//         return 0;
//     }
// }

// void debug(Waypoint p) {
//     std::cout << p.getX() << "\t" << p.getY() << "\t" << p.getTheta() << "\n";
// }