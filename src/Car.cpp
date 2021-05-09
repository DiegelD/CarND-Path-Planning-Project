
#include "Car.h"
#include "Cost.h"

#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>

void Car::update(double s_in, double d_in, double yaw_in, double speed_in)
{
    s = s_in;
    d = d_in;
    yaw = yaw_in;
    v = speed_in;
    //this->info();
}
// Initializes Vehicle
Car::Car(){}

Car::Car(int lane, double s, double v, double a, string state)
{
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;
}

Car::~Car() {}

vector<Car> Car::choose_next_state(vector<Car> &predictions)
{
    /**
   * Here you can implement the transition_function code from the Behavior 
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle 
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego 
   *   vehicle state.
   *
   * Functions:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing 
   *    a vehicle trajectory, given a state and predictions. Note that 
   *    trajectory vectors might have size 0 if no possible trajectory exists 
   *    for the state. 
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.

   */
    std::cout << " v1" << std::endl;
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<vector<Car> > final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        vector<Car> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0)
        {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }

    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);

    return final_trajectories[best_idx];
}

vector<string> Car::successor_states()
{
    // Provides the possible next states given the current state for the FSM
    //   discussed in the course, with the exception that lane changes happen
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if (state.compare("KL") == 0)
    {
        states.push_back("PLCL");
        states.push_back("PLCR");
    }
    else if (state.compare("PLCL") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (lane != 0)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    std::cout << " v2" << std::endl;
    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Car> Car::generate_trajectory(string state,
                                             vector<Car> &predictions)
{
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Car> trajectory;
    if (state.compare("CS") == 0)
    {
        trajectory = constant_speed_trajectory();
    }
    else if (state.compare("KL") == 0)
    {
        trajectory = keep_lane_trajectory(predictions);
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
    {
        trajectory = lane_change_trajectory(state, predictions);
    }
    else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
    {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    std::cout << " v3" << std::endl;
    return trajectory;
}
 
vector<double> Car::get_kinematics(  vector<Car> &predictions,
                                      int lane)
{
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    double max_velocity_accel_limit = this->max_acceleration + this->v;
    double new_position;
    double new_velocity;
    double new_accel;
    Car vehicle_ahead;
    Car vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead))
    {
        if (get_vehicle_behind(predictions, lane, vehicle_behind))
        {
            // must travel at the speed of traffic, regardless of preferred buffer
            new_velocity = vehicle_ahead.v;
        }
        else
        {
            double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = std::min(std::min(max_velocity_in_front,
                                             max_velocity_accel_limit),
                                    this->target_speed);
        }
    }
    else
    {
        new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
    new_position = this->s + new_velocity + new_accel / 2.0;
    std::cout << " v4" << std::endl;
    return {new_position, new_velocity, new_accel};
}

vector<Car> Car::constant_speed_trajectory()
{
    // Generate a constant speed trajectory.
    double next_pos = position_at(1);
    vector<Car> trajectory = {Car(this->lane, this->s, this->v, this->a, this->state),
                                  Car(this->lane, next_pos, this->v, 0, this->state)};
    std::cout << " v5" << std::endl;
    return trajectory;
}

vector<Car> Car::keep_lane_trajectory(vector<Car> &predictions)
{
    // Generate a keep lane trajectory.
    vector<Car> trajectory = {Car(lane, this->s, this->v, this->a, state)};
    vector<double> kinematics = get_kinematics(predictions, this->lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    trajectory.push_back(Car(this->lane, new_s, new_v, new_a, "KL"));
    std::cout << " v6" << std::endl;
    return trajectory;
}

vector<Car> Car::prep_lane_change_trajectory(string state,
                                                     vector<Car> &predictions)
{
    // Generate a trajectory preparing for a lane change.
    double new_s;
    double new_v;
    double new_a;
    Car vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Car> trajectory = {Car(this->lane, this->s, this->v, this->a,
                                          this->state)};
    vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind))
    {
        // Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
    }
    else
    {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        // Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1])
        {
            best_kinematics = next_lane_new_kinematics;
        }
        else
        {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    trajectory.push_back(Car(this->lane, new_s, new_v, new_a, state));
    std::cout << " v7" << std::endl;
    return trajectory;
}

vector<Car> Car::lane_change_trajectory(string state,
                                                vector<Car> &predictions)
{
    std::cout << " v7.0" << std::endl;
    // Generate a lane change trajectory.
    int new_lane = this->lane + lane_direction[state];
    vector<Car> trajectory;
    Car next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies
    //   that spot).
    for (int i = 0; i < predictions.size(); i++)
    {
        next_lane_vehicle = predictions[i];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane)
        {
            // If lane change is not possible, return empty trajectory.
            std::cout << " v7.1" << std::endl;
            return trajectory;
        }
    }
    trajectory.push_back(Car(this->lane, this->s, this->v, this->a, this->state));
    vector<double> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Car(new_lane, kinematics[0], kinematics[1],
                                 kinematics[2], state));
    std::cout << " v7.2" << std::endl;
    return trajectory;
}

void Car::increment(int dt = 1)
{
    this->s = position_at(dt);
}

float Car::position_at(int t)
{
    return this->s + this->v * t + this->a * t * t / 2.0;
}

bool Car::get_vehicle_behind(vector<Car> &predictions,
                                 int lane, Car &rCar)
{
    // Returns a true if a vehicle is found behind the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Car temp_vehicle;
    for (int i = 0; i < predictions.size(); i++)
    {
        temp_vehicle = predictions.at(i);
        if (temp_vehicle.lane == this->lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s)
        {
            max_s = temp_vehicle.s;
            rCar = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Car::get_vehicle_ahead( vector<Car> &predictions,
                                int lane, Car &rCar)
{
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int min_s = this->goal_s;
    bool found_vehicle = false;
    Car temp_vehicle;
    for (int i = 0; i < predictions.size(); i++)
    {
        temp_vehicle = predictions.at(i);
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s)
        {
            min_s = temp_vehicle.s;
            rCar = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Car> Car::generate_predictions(const vector<vector<double> > &sensor_fusion, Car &car)
{
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    // SONSOR_FUSION :The data format for each car is: [ id, x, y, vx, vy, s, d]. 
        // The id is a unique identifier for that car. The x, y values are in global map coordinates, 
        //  Finally s and d are the Frenet coordinates for that car.
    vector<Car> predictions;
    
    for (int i = 0; i < sensor_fusion.size(); i++)
    {   
        vector<int> predicted_lane {0,1,2};
        float d = sensor_fusion[i][6]; 
        // Check is objeckt is in the same line
        for( int j=0; j<predicted_lane.size(); j++ )
        {
            if (d < (2 + 4 * predicted_lane[j] + 2) && d > (2 + 4 * predicted_lane[j] - 2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx * vy * vy);
                double check_car_s = sensor_fusion[i][5];

                if ((check_car_s > (car.s-20)) && ((check_car_s - car.s) < 40)) // Take just cars in the closest enviroment into consideration
                {
                    predictions.push_back(Car(predicted_lane[j], check_car_s, check_speed, 0));
                }  
            }
        }
    }
    return predictions;
}




void Car::realize_next_state(vector<Car> &trajectory)
{
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    Car next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
    std::cout << " v8" << std::endl;
}