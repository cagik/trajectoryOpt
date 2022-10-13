#pragma once

#include <vector>
#include <tuple>
#include <cmath>

#define DELT_STEP 2

using namespace std;

typedef tuple<double, double, double, double> point4d;
typedef vector<tuple<double, double, double, double>> PathInfo;
typedef std::vector<std::pair<double, double>> obsInfo;

void RotatoByAxis(double _x, double _y, double tht, double x_axis, double y_axis, double &x_res, double &y_res);

class simData
{
public:
    simData();
    PathInfo Test1();
    obsInfo obs1();
};




