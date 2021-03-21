
#include "Car.h"
#include "Cost.h"

#include <iostream>
#include <algorithm>
#include <vector>

// Initializes Vehicle
Car::Car()
{
    double x = 0;
    double y = 0;
    double s = 0;
    double d = 0;
    double yaw = 0;
    double speed = 0;
    int lane = 1;
    int indended_lane = 1;
    int goal_lane =1;
    bool too_close = false;
    std::string current_state = "KL";
}

Car::~Car()
{
}

void Car::update(double x_in, double y_in, double s_in, double d_in, double yaw_in, double speed_in)
{
    x = x_in;
    y = y_in;
    s = s_in;
    d = d_in;
    yaw = yaw_in;
    speed = speed_in;
    //this->info();
}
void Car::info()
{
    cout << "current_lane                " << lane << " " << endl;
    cout << "Car.x   " << x << " " << endl;
    cout << "Car.y   " << y << " " << endl;
    cout << "Car.s   " << s << " " << endl;
    cout << "Car.d  " << d << " " << endl;
    cout << "Car.yaw   " << yaw << " " << endl;
    cout << "Car.speed   " << speed << " " << endl;
}

vector<string> Car::successor_states()
{
    vector<string> successor_states;

    successor_states.push_back("KL");
    successor_states.push_back(current_state);

    if (current_state.compare("KL") == 0)
    {
        successor_states.push_back("PLCL");
        successor_states.push_back("PLCR");
    }
    else if (current_state.compare("PLCL") == 0)
    {
        if (lane != 0)   // Not already very left lane
        {
            successor_states.push_back("PLCL");
            successor_states.push_back("LCL");
        }
    }
    else if (current_state.compare("PLCR") == 0)
    {
        if (lane != 2) // Not already very left lane
        {
            successor_states.push_back("PLCR");
            successor_states.push_back("LCR");
        }
    }
    // If state is "LCL" or "LCR", then just return "KL"
    return successor_states;
}

string Car::choose_next_state(Car &car, const vector<vector<double> > &sensor_fusion)
{
    /**:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory Lane
   * 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a lane change.
   *
   */
    vector<string> states = successor_states();
    cout << "****Car::choose_nect_state succesor_states****    "<< endl;
    float cost;
    vector<float> costs;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        std::cout << *it << ' ';
        // Change so that just when the call is there its getting calculated
        cost = calculate_cost(car, sensor_fusion);
        cout << "Cost   " << cost << endl;
        costs.push_back(cost);
    }
    cout << "****Car::choose_nect_state succesor_states  END****    " << endl;
    //vector<float>::iterator
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    cout << "----> Best State    " << states[best_idx] << endl;
    return states[best_idx]; // Change
}