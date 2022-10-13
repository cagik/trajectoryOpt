
#include "cartesian_planner.h"

namespace cartesian_planner {


bool CartesianPlanner::Plan(const Trajectory coarse_trajectory,  KDTree &obsTree, std::vector<box> &boxes, 
            Trajectory &result, pair<PointBoundry, PointBoundry> &tpb)
{
    States optimized;
    if(!opti_.OptimizeIteratively(coarse_trajectory, obsTree, boxes, optimized, tpb)) {
      std::cout << "Optimization failed" << std::endl;
//      return false;
    }

  
    std::vector<double> opti_x, opti_y, opti_v;
    Trajectory result_data;
    for(int i = 0; i < config_.nfe; i++) {
      TrajectoryPoint tp;
      tp.x = optimized.x[i];
      tp.y = optimized.y[i];
      tp.theta = optimized.theta[i];
      tp.velocity = optimized.v[i];
      tp.kappa = tan(optimized.phi[i]) / config_.vehicle.wheel_base;
      opti_x.push_back(tp.x);
      opti_y.push_back(tp.y);
      opti_v.push_back(tp.velocity);
      result_data.push_back(tp);
    }

    result = result_data;
    return true;
}

}
