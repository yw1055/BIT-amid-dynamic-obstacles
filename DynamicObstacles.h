#ifndef SRC_DYNAMICOBSTACLE_H
#define SRC_DYNAMICOBSTACLE_H
#include <cmath>
#include <iostream>
struct Dynamicxy {
    double x, y, radius;
    Dynamicxy(double x, double y, double radius): x(x), y(y), radius(radius){};
    Dynamicxy(){};
    double operator-(const Dynamicxy &d) {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }
};
struct real_location {
    double xl,yl, dTime;
    real_location(double x, double y, double t): xl(x), yl(y),dTime(t){};
};
struct movement {
    double heading, speed, x, y,mTime;
    int steps;
    movement(): heading(0),speed(0),steps(0){};
    movement(double x, double y, double heading, double speed, int step, double t):x(x), y(y), heading(heading),speed(speed),steps(step),mTime(t){};
};
struct DynamicObstacle {
    std::vector<movement> instructions;
    std::vector<real_location> location;
    double x, y, estimate_h, estimate_s, heading, speed,curTime;
    int remain, current, total;
    DynamicObstacle(int x, int y, double heading, double speed): x(x), y(y), estimate_h(0), estimate_s(0), heading(heading),speed(speed){};
    DynamicObstacle(int x, int y, int total): x(x), y(y), estimate_h(0), estimate_s(0), heading(0),speed(0), current(0), total(total){};
    DynamicObstacle(): estimate_h(0),estimate_s(0){};
    Dynamicxy getxy() {
        return (Dynamicxy(x, y, 1));
    };
    double operator-(const DynamicObstacle d) {
        return sqrt((x - d.x) * (x - d.x) + (y - d.y) * (y - d.y));
    }
    friend double operator-(Dynamicxy d, const DynamicObstacle &d1) {
        return sqrt((d.x - d1.x) * (d.x - d1.x) + (d.y - d1.y) * (d.y - d1.y));
    }
    friend double operator-(DynamicObstacle d, const Dynamicxy &d1) {
        return sqrt((d.x - d1.x) * (d.x - d1.x) + (d.y - d1.y) * (d.y - d1.y));
    }
};
#endif //SRC_DYNAMICOBSTACLE_H
