#ifndef __OBSTACLE_H__
#define __OBSTACLE_H__
#include <vector>
#include <cmath>
#include "DynamicObstacle.h"
using namespace std;
class Obstacle {
private:
    vector<DynamicObstacle> dyObs;
public:
    Obstacle();
    Obstacle(vector<DynamicObstacle> dyo): dyObs(dyo){};

    void addDynamicObstacle(DynamicObstacle &d);
    DynamicObstacle DyInfo(DynamicObstacle & dynamic, double t);
    double get_probability(DynamicObstacle &dynamic, double x, double y, double t);
    vector<DynamicObstacle> getDynamicObstacle();
};
#endif
