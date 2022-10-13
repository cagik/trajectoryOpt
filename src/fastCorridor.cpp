#include "fastCorridor.h"

using namespace std;

void fastCorridor::Init(std::vector<std::pair<double, double>> &left_boundary, std::vector<std::pair<double, double>> &right_boundary)
{
    pointVec leftTreeVec;
    point_t pt;
    for (size_t i = 0; i < left_boundary.size(); i++)
    {
        pt = {left_boundary[i].first, left_boundary[i].second};
        leftTreeVec.push_back(pt);
    }
    leftTree_ = KDTree(leftTreeVec);

    pointVec rightTreeVec;
    for (size_t i = 0; i < right_boundary.size(); i++)
    {
        pt = {right_boundary[i].first, right_boundary[i].second};
        rightTreeVec.push_back(pt);
    }
    rightTree_ = KDTree(rightTreeVec);

    front_margin_ = 5.0;
    rear_margin_ = 2.0;
    left_margin_ = -2.0;
    right_margin_ = 2.0;
    transverse_SafeDistance_ = 3.0;
    safeCoefficient_ = 0.5;
}

void fastCorridor::BuildCorridor(std::vector<std::tuple<double, double, double, double>> &referTraj)
{
    double margin = 0.5;

    boxes_.clear();
    boxes_.resize(referTraj.size());
    for (size_t i = 0; i < referTraj.size(); i++)
    {
        double x_ref = get<0>(referTraj[i]);
        double y_ref = get<1>(referTraj[i]);
        double theta_ref = get<2>(referTraj[i]);

        point_t pt = {x_ref, y_ref};
        auto left_res = leftTree_.nearest_point(pt);
        double left_dis = -1 * hypot(pt[0]- left_res[0], pt[1] - left_res[1]);
        boxes_[i].t_lb = left_dis > left_margin_ ? left_dis * safeCoefficient_ + transverse_SafeDistance_ : left_dis * safeCoefficient_;

        auto right_res = rightTree_.nearest_point(pt);
        double right_dis = hypot(pt[0] - right_res[0], pt[1] - right_res[1]);
        boxes_[i].t_ub = right_dis > right_margin_ ? right_dis * safeCoefficient_ : right_dis * safeCoefficient_ - transverse_SafeDistance_;

        if(boxes_[i].t_lb >= boxes_[i].t_ub){
            boxes_[i].t_lb = -1.0;
            boxes_[i].t_ub = 1.0;
        }

        double front_dis = 0.5;
        pair<double, double> leftFrontPoint_inertia = make_pair(boxes_[i].t_lb, front_dis);
        pair<double, double> rightFrontPoint_inertia = make_pair(boxes_[i].t_ub, front_dis);
        pair<double, double> leftFrontPoint_global;
        pair<double, double> rightFrontPoint_global;
        for (int j = 0; j < 5; j++)
        {
            
            double front_dis_tmp = front_dis + double(j * 1);
            leftFrontPoint_inertia.second = front_dis_tmp;
            rightFrontPoint_inertia.second = front_dis_tmp;
            leftFrontPoint_global.first = cos(theta_ref - M_PI / 2) * leftFrontPoint_inertia.first 
                                        - sin(theta_ref - M_PI / 2) * leftFrontPoint_inertia.second + x_ref;
            leftFrontPoint_global.second = sin(theta_ref - M_PI / 2) * leftFrontPoint_inertia.first 
                                        + cos(theta_ref - M_PI / 2) * leftFrontPoint_inertia.second + y_ref;
            rightFrontPoint_global.first = cos(theta_ref - M_PI / 2) * rightFrontPoint_inertia.first 
                                        - sin(theta_ref - M_PI / 2) * rightFrontPoint_inertia.second + x_ref;
            rightFrontPoint_global.second = sin(theta_ref - M_PI / 2) * rightFrontPoint_inertia.first 
                                        + cos(theta_ref - M_PI / 2) * rightFrontPoint_inertia.second + y_ref;
            
            point_t pt = {leftFrontPoint_global.first, leftFrontPoint_global.second};
            auto left_res = leftTree_.nearest_point(pt);
            double left_dis = hypot(pt[0] - left_res[0], pt[1] - left_res[1]);
            pt = {rightFrontPoint_global.first, rightFrontPoint_global.second};
            auto right_res = rightTree_.nearest_point(pt);
            double right_dis = hypot(pt[0] - right_res[0], pt[1] - right_res[1]);



            if(left_dis < margin || right_dis < margin){
                boxes_[i].l_ub = (front_dis + double(j - 1) * 1) * safeCoefficient_;
                break;
            }
            else{
                boxes_[i].l_ub = (front_dis + double(j) * 1) * safeCoefficient_;
            }
            
        }
        
        double rear_dis = -0.5;
        pair<double, double> leftRearPoint_inertia = make_pair(boxes_[i].t_lb, rear_dis);
        pair<double, double> rightRearPoint_inertia = make_pair(boxes_[i].t_ub, rear_dis);
        pair<double, double> leftRearPoint_global;
        pair<double, double> rightRearPoint_global;
        for (int j = 0; j < 3; j++)
        {
            double rear_dis_tmp = rear_dis - double(j * 1);
            leftRearPoint_inertia.second = rear_dis_tmp;
            rightRearPoint_inertia.second = rear_dis_tmp;
            leftRearPoint_global.first = cos(theta_ref - M_PI / 2) * leftRearPoint_inertia.first 
                                        - sin(theta_ref - M_PI / 2) * leftRearPoint_inertia.second + x_ref;
            leftRearPoint_global.second = sin(theta_ref - M_PI / 2) * leftRearPoint_inertia.first 
                                        + cos(theta_ref - M_PI / 2) * leftRearPoint_inertia.second + y_ref;
            rightRearPoint_global.first = cos(theta_ref - M_PI / 2) * rightRearPoint_inertia.first 
                                        - sin(theta_ref - M_PI / 2) * rightRearPoint_inertia.second + x_ref;
            rightRearPoint_global.second = sin(theta_ref - M_PI / 2) * rightRearPoint_inertia.first 
                                        + cos(theta_ref - M_PI / 2) * rightRearPoint_inertia.second + y_ref;
            
            point_t pt = {leftRearPoint_global.first, leftRearPoint_global.second};
            auto left_res = leftTree_.nearest_point(pt);
            double left_dis = hypot(pt[0] - left_res[0], pt[1] - left_res[1]);
            pt = {rightRearPoint_global.first, rightRearPoint_global.second};
            auto right_res = rightTree_.nearest_point(pt);
            double right_dis = hypot(pt[0] - right_res[0], pt[1] - right_res[1]);

            if(left_dis < margin || right_dis < margin){
                boxes_[i].l_lb = (rear_dis - double(j - 1) * 1) * safeCoefficient_; 
            }
            else{
                boxes_[i].l_lb = (rear_dis - double(j) * 1) * safeCoefficient_; 
            }

            
        }
        

        if(boxes_[i].l_lb >= boxes_[i].l_ub)
        {

            boxes_[i].l_lb = -1.0;
            boxes_[i].l_ub = 1.0;
        }
    }
}

vector<vector<pair<double, double>>> fastCorridor::getBox_global(std::vector<std::tuple<double, double, double, double>> &referTraj)
{
    vector<vector<pair<double, double>>> res;
    for (size_t i = 0; i < referTraj.size(); i++)
    {
        double x_ref = get<0>(referTraj[i]);
        double y_ref = get<1>(referTraj[i]);
        double theta_ref = get<2>(referTraj[i]);

        pair<double, double> leftFrontPoint_global;
        pair<double, double> rightFrontPoint_global;
        pair<double, double> leftRearPoint_global;
        pair<double, double> rightRearPoint_global;

        leftFrontPoint_global.first = cos(theta_ref - M_PI / 2) * boxes_[i].t_lb
                                    - sin(theta_ref - M_PI / 2) * boxes_[i].l_ub + x_ref;
        leftFrontPoint_global.second = sin(theta_ref - M_PI / 2) * boxes_[i].t_lb
                                    + cos(theta_ref - M_PI / 2) * boxes_[i].l_ub + y_ref;
        rightFrontPoint_global.first = cos(theta_ref - M_PI / 2) * boxes_[i].t_ub
                                    - sin(theta_ref - M_PI / 2) * boxes_[i].l_ub + x_ref;
        rightFrontPoint_global.second = sin(theta_ref - M_PI / 2) * boxes_[i].t_ub
                                    + cos(theta_ref - M_PI / 2) * boxes_[i].l_ub + y_ref;
        
        leftRearPoint_global.first = cos(theta_ref - M_PI / 2) * boxes_[i].t_lb
                                    - sin(theta_ref - M_PI / 2) * boxes_[i].l_lb + x_ref;
        leftRearPoint_global.second = sin(theta_ref - M_PI / 2) * boxes_[i].t_lb
                                    + cos(theta_ref - M_PI / 2) * boxes_[i].l_lb + y_ref;
        rightRearPoint_global.first = cos(theta_ref - M_PI / 2) * boxes_[i].t_ub
                                    - sin(theta_ref - M_PI / 2) * boxes_[i].l_lb + x_ref;
        rightRearPoint_global.second = sin(theta_ref - M_PI / 2) * boxes_[i].t_ub
                                    + cos(theta_ref - M_PI / 2) * boxes_[i].l_lb + y_ref;
        
        res.push_back({leftFrontPoint_global, rightFrontPoint_global, leftRearPoint_global, rightRearPoint_global});
    }
    
    return res;
}