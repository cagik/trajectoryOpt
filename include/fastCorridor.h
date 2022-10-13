#ifndef FAST_CORRIDOR_H
#define FAST_CORRIDOR_H

#include <iostream>
#include <tuple>
#include <vector>
#include <cmath>

#include "KDTree.hpp"

struct box
{
    double l_ub, l_lb, t_lb, t_ub;
};


class fastCorridor{
public:
    void Init(std::vector<std::pair<double, double>> &left_boundary, std::vector<std::pair<double, double>> &right_boundary);
    void BuildCorridor(std::vector<std::tuple<double, double, double, double>> &referTraj);
    std::vector<std::vector<std::pair<double, double>>> getBox_global(std::vector<std::tuple<double, double, double, double>> &referTraj);

    std::vector<box> boxes_;

private:

    double front_margin_, rear_margin_, right_margin_, left_margin_;
    double transverse_SafeDistance_;
    double safeCoefficient_;
    KDTree leftTree_;
    KDTree rightTree_;
};


#endif