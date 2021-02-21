#ifndef BEHAVIOURPLANNER_H
#define BEHAVIOURPLANNER_H

#include <vector>
using std::vector;

class BehaviourPlanner
{
private:
    bool lane_free_one_;
    bool lane_free_two_;
    bool lane_free_three_;

    void lane_check_around_(vector<vector<double>> sensor_fusion, Car &car, int prev_size);

public:
    BehaviourPlanner();
    ~BehaviourPlanner();

    bool KeepLane;
    // vector<Vehicle::State> get_sucessor_State(Vehicle &vehicle);
    void update(Car &car);
    void lane_check(vector<vector<double>> sensor_fusion, Car &car, int prev_size);
};

#endif
