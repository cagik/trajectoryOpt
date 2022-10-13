#include "corridor.h"

void corridor::initCorridor(KDTree tree, double x_min, double x_max, double y_min, double y_max)
{
    obstacleTree_ = tree;
    x_min_ = x_min;
    x_max_ = x_max;
    y_min_ = y_min;
    y_max_ = y_max;
}


void corridor::update(const vector<pair<double, double>> traj, double margin, bool updateMode){
    traj_ = traj;
    updateObsBox(margin, updateMode);
}

bool corridor::isBoxInBoundary(const vector<double>& box)
{
    return  box[0] > x_min_ &&
            box[1] > y_min_ &&
            box[2] < x_max_ &&
            box[3] < y_max_;
}

bool corridor::isPointInBox(const pair<double,double> point, const vector<double>& box)
{
    return  point.first > box[0] &&
            point.second > box[1] &&
            point.first < box[2] &&
            point.second < box[3];
}

bool corridor::isObstacleInBox(vector<double>& box, double margin){
    double x,y;
    double dis;
    double dx,dy;
    point_t pt;

    if(box[0] > box[2]){
        swap(box[0], box[2]);
    }

    if(box[1] > box[3]){
        swap(box[1], box[3]);
    }

    dx = (box[2] - box[0])/5;
    dy = (box[3] - box[1])/5;

    for(double i = box[0]; i < box[2] + 0.02; i = i + dx){
        for(double j = box[1]; j < box[3] + 0.02; j = j + dy ){
            x = i - 0.02;
            y = j - 0.02;

            pt.clear();
            pt.push_back(x);
            pt.push_back(y);
            
            auto res = obstacleTree_.nearest_point(pt);
            dis = sqrt(pow(pt[0] - res[0], 2) + pow(pt[1] - res[1], 2));
            if(dis < margin){
                return true;
            }
        }
    }
    return false;
}

void corridor::expandBox(vector<double>& box, double margin){
    vector<double> box_cand, box_update;
    vector<int> axis_cand{0, 1, 2, 3};

    int i = -1;
    int axis;
    while(!axis_cand.empty()){
        box_cand = box;
        box_update = box;

        while(!isObstacleInBox(box_update, margin) && isBoxInBoundary(box_update)){
            i++;
            if(i >= axis_cand.size()){
                i = 0;
            }
            axis = axis_cand[i];

            box = box_cand;
            box_update = box_cand;

            if(axis < 2){
                box_update[axis + 2] = box_cand[axis];
                box_cand[axis] = box_cand[axis] - 0.3;
                box_update[axis] = box_cand[axis];
            }
            else{
                box_update[axis - 2] = box_cand[axis];
                box_cand[axis] = box_cand[axis] + 0.3;
                box_update[axis] = box_cand[axis];
            }
        }
        axis_cand.erase(axis_cand.begin() + i);
        if(i > 0){
            i --;
        }
        else{
            i = axis_cand.size()-1;
        }
    }
}

void corridor::changeRef(vector<double>& box, double margin){
    int flag = true;
    vector<double> tmpBox;
    vector<pair<double, double>> dir = { {-1, 1}, {0, 1}, {1, 1},
                                         {-1, 0},         {1, 0},
                                         {-1, -1}, {0, -1}, {1, -1}
                                        };
    int iter = 0;
    double stepLength = 0.3;
    while(flag){
        for(size_t i = 0; i < dir.size(); i++){
            tmpBox = box;
            tmpBox[0] += stepLength * iter * dir[i].first;
            tmpBox[1] += stepLength * iter * dir[i].second;
            tmpBox[2] += stepLength * iter * dir[i].first;
            tmpBox[3] += stepLength * iter * dir[i].second;
            if(!isObstacleInBox(tmpBox, margin)){
                box = tmpBox;
                flag = false;
                break;
            }
        }
        iter++;
    }
}

bool corridor::isPointInObstacle(double x, double y, double margin)
{
    point_t tmp;
    tmp.clear();
    tmp.push_back(x);
    tmp.push_back(y);
    auto res = obstacleTree_.nearest_point(tmp);
    double dis = hypot(res[0] - tmp[0], res[1] - tmp[1]);
    if(dis < margin){
        return true;
    }
    else{
        return false;
    }
}

void corridor::updateObsBox(double margin, bool fastUpdateMode)
{
    vector<double> box_prev{0,0,0,0};
    vector<double> box;
    double x, y, x_next, y_next;

    for(size_t i = 0; i < traj_.size()-1; i++){

        x = traj_[i].first;
        y = traj_[i].second; 

        x_next = traj_[i+1].first;
        y_next = traj_[i+1].second;

        box.clear();

        if(isPointInBox(make_pair(x, y), box_prev))
        {
            boxes.push_back(box_prev);
            continue;
        }

        cout << "NO." << i << endl;

        //init box
        if(fastUpdateMode == true){
            box.emplace_back(min(x,x_next));
            box.emplace_back(min(y,y_next));
            box.emplace_back(max(x,x_next));
            box.emplace_back(max(y,y_next));
        }
        else{
            box.emplace_back(x - 0.5);
            box.emplace_back(y - 0.5);
            box.emplace_back(x + 0.5);
            box.emplace_back(y + 0.5);
        }


        // if(isPointInObstacle(x, y, 1.5) || isObstacleInBox(box, margin)){
        //     changeRef(box, margin);
        // }

        cout << "NO." << i << endl;

        expandBox(box, margin);

        

        if(box[0] > box[2]){
            swap(box[0], box[2]);
        }
        if(box[1] > box[3]){
            swap(box[1], box[3]);
        } 

        vis_boxes.push_back(box);
        boxes.push_back(box);
        box_prev = box;
    }
    boxes.push_back(box_prev);
}

