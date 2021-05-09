#ifndef CAR_H
#define CAR_H

#include <string>
#include <vector>
#include <map>

using std::string;
using std::vector;
using namespace std;

class Car
{
public:
    // Constructors
    Car();

    Car(int lane, double s, double v, double a, string state = "CS");

    // Destructor
    virtual ~Car();

    // Vehicle functions
    void update(double car_s, double car_d, double car_yaw, double car_speed);
    vector<Car> choose_next_state(vector<Car> &predictions);

    vector<string> successor_states();

    vector<Car> generate_trajectory(string state,
                                         vector<Car> &predictions);

    vector<double> get_kinematics(vector<Car> &predictions, int lane);

    vector<Car> constant_speed_trajectory();

    vector<Car> keep_lane_trajectory( vector<Car> &predictions);

    vector<Car> lane_change_trajectory(string state,
                                           vector<Car> &predictions);

    vector<Car> prep_lane_change_trajectory(string state,
                                                vector<Car> &predictions);

    void increment(int dt);

    float position_at(int t);

    bool get_vehicle_behind( vector<Car> &predictions, int lane,
                            Car &rCar);

    bool get_vehicle_ahead( vector<Car> &predictions, int lane,
                           Car &rCar);

    vector<Car> generate_predictions(const vector<vector<double> > &sensor_fusion, Car &car);

    void realize_next_state(vector<Car> &trajectory);

    map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

    int L = 1;

    int preferred_buffer = 6; // impacts "keep lane" behavior.

    int lane, s, goal_lane, goal_s, lanes_available;

    double x, y, yaw, v, d,target_speed, a, max_acceleration;

    string state;

    bool too_close;
};
#endif