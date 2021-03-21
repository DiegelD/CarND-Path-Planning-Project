#ifndef BEHAVIOURPLANNER_H
#define BEHAVIOURPLANNER_H

#include <vector>
#include "Car.h"

using std::vector;

class BehaviourPlanner
{
private:
    bool close_object_CR_;
    bool close_object_LL_;
    bool close_object_LM_;
    bool close_object_LR_;

void lane_check_around_(const vector<vector<double> > & sensor_fusion, Car &car, int prev_size);

public:
    BehaviourPlanner();
    ~BehaviourPlanner();

    enum BP_State { Keep_Lane, PLCL, PLCR, LCL, LCR };
    BP_State state;

    bool lane_free_one;
    bool lane_free_two;
    bool lane_free_three;
    bool KeepLane;
    // vector<Vehicle::State> get_sucessor_State(Vehicle &vehicle);
    void update(Car car);
    void lane_check(const vector<vector<double> > & sensor_fusion, Car &car, int prev_size);
    void State_Machine();
};



#endif
