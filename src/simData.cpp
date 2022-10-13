#include "simData.h"

void RotatoByAxis(double _x, double _y, double tht, double x_axis, double y_axis, double &x_res, double &y_res){
    x_res = (_x - x_axis)*cos(tht) - (_y - y_axis)*sin(tht) + x_axis;
    y_res = (_x - x_axis)*sin(tht) + (_y - y_axis)*cos(tht) + y_axis;
}

simData::simData(/* args */)
{
}

PathInfo simData::Test1(){
    PathInfo path;
    double x0 = 0, y0 = 0, h0 = 0;
  // S: 直行20m，间隔1m，起点(0,0)
    for(double i = 0.0; i < 20.0; i+=DELT_STEP){
        point4d tmp;
        get<0>(tmp) = x0 + i;
        get<1>(tmp) = y0;
        get<2>(tmp) = h0;
        get<3>(tmp) = 0;
        path.push_back(tmp);
    }
    // L turn：半径20m，起点(20, 0), 转90度，间隔DELT_STEPm
    double r = 20.0;
    double l = 0.5*M_PI*r / DELT_STEP;
    int r_step = int(l);
    double tht_step = 0.5*M_PI / r_step;

    for (int i = 0; i < r_step; i++){
        point4d tmp_last = path.at(path.size() - 1);
        double x_last, y_last;
        x_last = get<0>(tmp_last);
        y_last = get<2>(tmp_last);
        double x, y;
        double _x = 20.0, _y = 0.0;
        double x_axis = 20.0, y_axis = 20.0;
        RotatoByAxis(_x, _y, i*tht_step, x_axis, y_axis, x, y);
        point4d tmp;
        get<0>(tmp) = x;
        get<1>(tmp) = y;
        get<2>(tmp) = (i*tht_step)*180.0 / M_PI;;
        get<3>(tmp) = 1/20.0;
        path.push_back(tmp);
    }
    // R turn: 半径20m，起点(40, 20)，旋转-90，间隔1m
    for (int i = 0; i < r_step; i++){
        point4d tmp_last = path.at(path.size() - 1);
        double x_last, y_last;
        x_last = get<0>(tmp_last);
        y_last = get<2>(tmp_last);
        double x, y;
        double _x = 40.0, _y = 20.0;
        double x_axis = 60.0, y_axis = 20.0;
        RotatoByAxis(_x, _y, -i*tht_step, x_axis, y_axis, x, y);
        point4d tmp;
        get<0>(tmp) = x;
        get<1>(tmp) = y;
        get<2>(tmp) = 90 + (-i*tht_step)*180.0 / M_PI;
        get<3>(tmp) = -1/20.0;
        path.push_back(tmp);
    }
    // S: 直行40m，间隔DELT_STEPm，起点(60, 40)
    for(double i = 0.0; i < 20.0; i+=DELT_STEP){
        point4d tmp;
        get<0>(tmp) = 60 + i;
        get<1>(tmp) = 40;
        get<2>(tmp) = h0;
        get<3>(tmp) = 0;
        path.push_back(tmp);
        path.push_back(tmp);
    }

    return path;

}

obsInfo simData::obs1(){
    std::vector<std::pair<double,double>> obs;
    
    //1
    double x = 40, y = 10;
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        y-=0.5;
    }
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        x+=0.5;
    }
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        y+=0.5;
    }
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        x-=0.5;
    }
    
    x = 40;
    y = 30;
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        y+=0.5;
    }
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        x-=0.5;
    }
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        y-=0.5;
    }
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        x+=0.5;
    }
    
    x = 28;
    y = 12;
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        y-=0.5;
    }
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        x+=0.5;
    }
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        y+=0.5;
    }
    for(int i = 0; i < 5; i++){
        obs.push_back(std::make_pair(x,y));
        x-=0.5;
    }
    

    return obs;

}