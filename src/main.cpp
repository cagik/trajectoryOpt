#include "dataStream.h"
#include "trajectory.h"
#include "fastCorridor.h"
#include "cartesian_planner_config.h"
#include "cartesian_planner.h"
#include "simData.h"


KDTree getKDTree(const std::vector<std::pair<double, double>> &obs){
    pointVec Points_vec;
    point_t pt;
    for(size_t i = 0; i < obs.size(); ++i)
    {
        pt.push_back(obs[i].first);
        pt.push_back(obs[i].second);
        Points_vec.push_back(pt);
        pt.clear();
    }
    KDTree myTree(Points_vec);
    return myTree;
}

vector<double> getDistance(KDTree &tree, vector<tuple<double, double, double, double>> &refPath)
{
    vector<double> distance;
    for(auto &point: refPath){
        point_t tmp;
        tmp.push_back(get<0>(point));
        tmp.push_back(get<1>(point));
        auto res = tree.nearest_point(tmp);
        double dis_tmp = hypot(res[0] - tmp[0], res[1] - tmp[1]);
        distance.push_back(dis_tmp);
        tmp.clear();
    }
    return distance;
}

vector<double> getDistance(KDTree &tree, cartesian_planner::Trajectory &optPath){
    vector<double> distance;
    for(auto &point: optPath){
        point_t tmp;
        tmp.push_back(point.x);
        tmp.push_back(point.y);
        auto res = tree.nearest_point(tmp);
        double dis_tmp = hypot(res[0] - tmp[0], res[1] - tmp[1]);
        distance.push_back(dis_tmp);
        tmp.clear();
    }
    return distance;
}

void CartesianPlannerTest(){

    simData SimData;
    vector<tuple<double, double, double, double>> ref_pts_4d;

    std::vector<std::pair<double, double>> left_boundary_pts;
    std::vector<std::pair<double, double>> right_boundary_pts;
    std::vector<std::pair<double, double>> obs_pts;
    pair<cartesian_planner::PointBoundry, cartesian_planner::PointBoundry> twoPointBoundry;
    cartesian_planner::mapBoundary mapboundry = {10000, -10000, 10000, -10000};

    dataStream::ReadTxtEdgeData("./testData/4/lef_obsfile.txt", left_boundary_pts);
    dataStream::ReadTxtEdgeData("./testData/4/rig_obsfile.txt", right_boundary_pts);
    dataStream::ReadTxtPathData("./testData/4/referPath_opt.txt", ref_pts_4d);

    obs_pts.insert(obs_pts.end(), left_boundary_pts.begin(), left_boundary_pts.end());
    obs_pts.insert(obs_pts.end(), right_boundary_pts.begin(), right_boundary_pts.end());

    KDTree obsTree = getKDTree(obs_pts);

    // obs_pts = SimData.obs1();
    // ref_pts_4d = SimData.Test1();

    cartesian_planner::Trajectory coarseTraj;
    cartesian_planner::calReferTraj(coarseTraj, ref_pts_4d, twoPointBoundry, mapboundry);

    cout << "test " << twoPointBoundry.first.theta << endl;

    fastCorridor fC;
    fC.Init(left_boundary_pts, right_boundary_pts);
    fC.BuildCorridor(ref_pts_4d);
    std::vector<std::vector<std::pair<double, double>>> boxes_global = fC.getBox_global(ref_pts_4d);

    dataStream::PathDataToTxt("./resultData/referPath.txt", ref_pts_4d);
    dataStream::obsDataToTxt("./resultData/left_boundary_file.txt", left_boundary_pts);
    dataStream::obsDataToTxt("./resultData/right_boundary_file.txt", right_boundary_pts);
    dataStream::obsDataToTxt("./resultData/obsfile.txt", obs_pts);
    dataStream::BoxDataToTxt("./resultData/fastboxfile.txt", boxes_global);

    cartesian_planner::CartesianPlannerConfig config_;
    config_.nfe = coarseTraj.size();
    //config_.tf = abs(config_.nfe * 0.4) + 1;
    std::shared_ptr<cartesian_planner::CartesianPlanner> planner_;
    planner_ = std::make_shared<cartesian_planner::CartesianPlanner>(config_);
    cartesian_planner::Trajectory result;

    planner_->Plan(coarseTraj, obsTree, fC.boxes_, result, twoPointBoundry);


    float next_x, next_y, now_x, now_y;
    for(std::size_t i = 0; i< result.size() ; i++)
    {
        double iTemp = 0;
        double lengthTmp = 0;
        if(i < 3)
        {
            result[i].kappa = 0;
        }
        else if(i < result.size() - 2)
        {
            for(int n = i - 2; n < i + 1; n++){
                float sub = result[n + 1].theta * 180 / M_PI - result[n].theta * 180 / M_PI;
                iTemp += sub;
                now_x = result[n].x;
                now_y = result[n].y;
                next_x = result[n+1].x;
                next_y = result[n+1].y;
                double dis_tmp = hypot(next_x - now_x, next_y - now_y);
                lengthTmp += dis_tmp;
            }
            if (iTemp > 180) iTemp = iTemp - 360;
            if (iTemp < -180) iTemp = iTemp + 360;
            result[i].kappa = iTemp * M_PI / 180 / lengthTmp;
        }
        else{
            result[i].kappa = 0;
        }
    }

    for(std::size_t i = 0; i< ref_pts_4d.size() ; i++)
    {
        double iTemp = 0;
        double lengthTmp = 0;
        if(i < 3)
        {
            get<3>(ref_pts_4d[i]) = 0;
        }
        else if(i < ref_pts_4d.size() - 2)
        {
            for(int n = i - 2; n < i + 1; n++){
                float sub = get<2>(ref_pts_4d[n + 1]) - get<2>(ref_pts_4d[n]);
                iTemp += sub;
                now_x = get<0>(ref_pts_4d[n]);
                now_y = get<1>(ref_pts_4d[n]);
                next_x = get<0>(ref_pts_4d[n + 1]);
                next_y = get<1>(ref_pts_4d[n + 1]);
                double dis_tmp = hypot(next_x - now_x, next_y - now_y);
                lengthTmp += dis_tmp;
            }
            if (iTemp > 180) iTemp = iTemp - 360;
            if (iTemp < -180) iTemp = iTemp + 360;
            get<3>(ref_pts_4d[i]) = iTemp * M_PI / 180 / lengthTmp;
        }
        else{
            get<3>(ref_pts_4d[i]) = 0;
        }
    }


    dataStream::TrajDataToTxt("./resultData/optTraj.txt", result);

    vector<double> refDistance = getDistance(obsTree, ref_pts_4d);
    vector<double> optDisTance = getDistance(obsTree, result);

    dataStream::distanceDataToTxt("./resultData/distance.txt", refDistance, optDisTance);

    double ref_sum_dis = 0.0;
    double opt_sum_dis = 0.0;
    for (size_t i = 1; i < ref_pts_4d.size(); i++)
    {
        double ref_dis_tmp = hypot(get<0>(ref_pts_4d[i]) - get<0>(ref_pts_4d[i - 1]), get<1>(ref_pts_4d[i]) - get<1>(ref_pts_4d[i - 1]));
        double opt_dis_tmp = hypot(result[i].x - result[i-1].x, result[i].y - result[i-1].y);
        ref_sum_dis += ref_dis_tmp;
        opt_sum_dis += opt_dis_tmp;
    }

    cout << "result ref_dis: " << ref_sum_dis << endl;
    cout << "result opt_dis: " << opt_sum_dis << endl;
    
}

int main(){
    CartesianPlannerTest();
    return 0;
}