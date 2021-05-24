#include "Car.h"
#include "Cost.h"

#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <string>

void Car::update(double x_in, double y_in, double s_in, double d_in, double yaw_in, double speed_in)
{ 
    x = x_in;
    y = y_in;
    s = s_in;
    d = d_in;
    yaw = yaw_in;
    waste_debug = speed_in;
    //this->info();
}
// Initializes Vehicle
Car::Car(){
    this->lane = 1;
    this->s = 0;
    this->v = 0;
    this->a = 0;
    this->state = "KL";
    max_acceleration = 0.224;
    this->target_speed = 49.5;
}

Car::Car(int lane, double s, double v, double a, string state)
{
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = 0.224;
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
    vector<vector<Car>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it)
    {
        std::cout << " v1 states" << *it << std::endl;
        vector<Car> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0)
        {
        
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            std::cout << " v1 costs" << cost << std::endl;
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
        if (lane != 0)
        {
            states.push_back("PLCL");
            states.push_back("LCL");
            
        }
    }
    else if (state.compare("PLCR") == 0)
    {
        if (lane != lanes_available - 1)
        {
            states.push_back("PLCR");
            states.push_back("LCR");
            
        }
    }
    //std::cout << " v2" << std::endl;
    // If state is "LCL" or "LCR", then just return "KL"
    return states;
}

vector<Car> Car::generate_trajectory(string state,
                                             vector<Car> &predictions)
{
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Car> trajectory;
    std::cout << " v3" << std::endl;
    std::cout << " state" << state << std::endl;

    if (state.compare("KL") == 0)
    {
        trajectory = keep_lane_trajectory(predictions);
        //std::cout << " 3.2" << std::endl;
    }
    else if (state.compare("LCL") == 0 || state.compare("LCR") == 0)
    {
        trajectory = lane_change_trajectory(state, predictions);
        //std::cout << " 3.3" << std::endl;
    }
    else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0)
    {
        trajectory = prep_lane_change_trajectory(state, predictions);
        //std::cout << " 3.4" << std::endl;
    }
    return trajectory;
}
 
vector<double> Car::get_kinematics(  vector<Car> &predictions,
                                      int lane)
{
    // Gets next timestep kinematics (position, velocity, acceleration)
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    //std::cout << " v4" << std::endl;
    double max_velocity_accel_limit = this->max_acceleration + this->v;
    double min_velocity_accel_limit = this->v - this->max_acceleration;
    double new_velocity;
    double new_accel;
    Car vehicle_ahead;
    Car vehicle_behind;
    double nix = 0;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead))
    {
        std::cout << " v4.1" << std::endl;
        if (get_vehicle_behind(predictions, lane, vehicle_behind))
        {
            // must travel at the speed of traffic, regardless of preferred buffer
            new_velocity = vehicle_ahead.v;
            std::cout << " v4.1.1" << std::endl;
            std::cout << " vehicle_ahead.v " << vehicle_ahead.v << std::endl;
            if (vehicle_ahead.v >= this->v)  // acelearation +- depending on the situation
            {
                new_velocity = std::min(std::min(vehicle_ahead.v,
                                                 max_velocity_accel_limit),
                                        this->target_speed);
                std::cout << " new velocity  " << new_velocity << std::endl;
            }
            else
            {
                new_velocity = std::max(std::max(vehicle_ahead.v,
                                                 min_velocity_accel_limit),
                                        nix);
                std::cout << " new velocity  " << new_velocity << std::endl;
            }
        }
        else
        {
            double max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a); // Does not work due to no real time simulation
            new_velocity = std::min(std::min(max_velocity_in_front,
                                             max_velocity_accel_limit),
                                    this->target_speed);
            std::cout << " v4.1.2" << std::endl;
            std::cout << " max_velocity_in_front" << max_velocity_in_front << std::endl;
            std::cout << " max_velocity_accel_limit " << max_velocity_accel_limit << std::endl;
            std::cout << " a " << this->a << std::endl;
        }
    }
    else
    {
        new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
        std::cout << " v4.2" << std:: endl;
    }

    new_accel = new_velocity - this->v; // Equation: (v_1 - v_0)/t = acceleration
    double new_position = this->s + new_velocity + new_accel / 2.0;
    
    return {new_position, new_velocity, new_accel};
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
    //std::cout << " v6" << std::endl;
    std::cout << " v6 new v " << new_v  << std::endl;
    return trajectory;
}

vector<Car> Car::prep_lane_change_trajectory(string state,
                                                     vector<Car> &predictions)
{
    // Generate a trajectory preparing for a lane change.
    std::cout << " v7" << std::endl;
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
        // std::cout << " v7.1 new_v" << new_v << std::endl;
    }
    else
    {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //std::cout << " v7.2" <<  std::endl;
        // Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1])
        {
            best_kinematics = next_lane_new_kinematics;
            //std::cout << " v7.3" << std::endl;
        }
        else
        {
            best_kinematics = curr_lane_new_kinematics;
            //std::cout << " v7.4 v: " << best_kinematics[1] << std::endl;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
        //std::cout << " v7.5 new_v " << new_v << std::endl;
    }

    trajectory.push_back(Car(this->lane, new_s, new_v, new_a, state));
    //std::cout << " v7" << std::endl;
    return trajectory;
}

vector<Car> Car::lane_change_trajectory(string state,
                                                vector<Car> &predictions)
{
    std::cout << " v8.0" << std::endl;
    // Generate a lane change trajectory.
    int new_lane = this->lane + lane_direction[state];
    vector<Car> trajectory;
    Car next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies
    //   that spot).
    for (int i = 0; i < predictions.size(); i++)
    {
        next_lane_vehicle = predictions[i];
        if (((next_lane_vehicle.s > (this->s - 10)) && ((next_lane_vehicle.s - this->s) < 20)) && next_lane_vehicle.lane == new_lane)
        {
            // If lane change is not possible, return empty trajectory.
            std::cout << " v8.1" << std::endl;
            return trajectory;
        }
    }
    trajectory.push_back(Car(this->lane, this->s, this->v, this->a, this->state));
    vector<double> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Car(new_lane, kinematics[0], kinematics[1],
                                 kinematics[2], state));

    std::cout << " v8.2" << std::endl;
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
    //std::cout << " v4.vah" << std::endl;
    double  min_s = 6945.554;  // Distance max_s from Cost function
    bool found_vehicle = false;
    Car temp_vehicle;
    for (int i = 0; i < predictions.size(); i++)
    {
        temp_vehicle = predictions.at(i);
        //std::cout << " temp_vehicle.lane" << temp_vehicle.lane << " This Lane "<< this->lane << std::endl;
        //std::cout << " temp_vehicle.s" << temp_vehicle.s << " This s " << this->s << " min s" << min_s << std::endl;
        //std::cout << " min.s" << temp_vehicle.s << " This s " << this->s << std::endl;
        if (temp_vehicle.lane == this->lane && temp_vehicle.s > (this->s-20) && temp_vehicle.s < min_s)  // -20 are buffer or better predict the other s
        {
            min_s = temp_vehicle.s;
            rCar = temp_vehicle;
            found_vehicle = true;
            //std::cout << " v4.vah found" << std::endl;
        }
    }
    return found_vehicle;
}

vector<Car> Car::generate_predictions(const vector<vector<double>> &sensor_fusion, Car &car, int &prev_size)
{
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    // SONSOR_FUSION :The data format for each car is: [ id, x, y, vx, vy, s, d]. 
        // The id is a unique identifier for that car. The x, y values are in global map coordinates, 
        //  Finally s and d are the Frenet coordinates for that car.
    vector<Car> predictions;
    
    for (int i = 0; i < sensor_fusion.size(); i++)
    {   
        vector<int> predicted_lane{0,1,2};
        float d = sensor_fusion[i][6]; 
        // Check is objeckt is in the same line
        for( int j=0; j<predicted_lane.size(); j++ )
        {
    
            if (d < (2 + 4 * predicted_lane[j] + 2) && d > (2 + 4 * predicted_lane[j] - 2))
            {
                double offset = 0;
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx + vy * vy);
                double check_car_s = sensor_fusion[i][5] + offset; // to compensate the simulator uncertainies
                //std::cout << " check_car_s " << check_car_s << std::endl;
                ;
                check_car_s += ((double)prev_size * .02 * check_speed);  //Speed Update due to simulation buffer
                //std::cout << "Lane " << predicted_lane[j] << "  After check_car_s " << check_car_s << std::endl;
                ;

                if ((check_car_s > (this->s - 5)) && ((check_car_s - this->s) < 50)) // Take just cars in the closest enviroment into consideration
                {
                    predictions.push_back(Car(predicted_lane[j], check_car_s, check_speed, 0));
                    std::cout << " Otheres Vehcile Lane: " << predicted_lane[j] << " Speed " << check_speed
                              << " vx " << vx << " vy " << vy << " s " << check_car_s << " This Car s " << this->s
                              << std::endl;
                }  
            }
        }
    }
    return predictions;
}





void Car::realize_next_state(vector<Car> &trajectory)
{
    // Sets state and kinematics for ego vehicle using the last state of the trajectory.
    //Logger::instance().log("trajectory", Logger::Debug);
    // std::cout << " v8" << std::endl;
    //std::cout << " car lane: " << this->lane << std::endl;

    Car next_state = trajectory[1];
    this->state = next_state.state;
    this->v = next_state.v;
    this->lane = next_state.lane;
    //this->s = next_state.s;
    this->a = next_state.a;

    std::cout << " next_state.lane: " << next_state.lane << std::endl;
    std::cout << " next_state.v: " << next_state.v << std::endl;
    std::cout << " this v" << this -> v << std::endl;
    //std::cout << " next_state.a: " << next_state.a << std::endl;
    /*
    if (next_state.v > 49.5)
    {
        next_state.v = 49.5;
        std::cout << " 8.1" << std::endl;
    }

    if ((next_state.v > this->v) && (next_state.v < 49.5))
    {
        this->v += .224;
        std::cout << " 8.2" << std::endl;
    }
    else if ((next_state.v < this->v) && (next_state.v > 0.5)) // 1 avid back driving
    {
        this->v -= .224;
        std::cout << " 8.3" << std::endl;
    }
    */

    //std::cout << " this v" << this->v << std::endl;
}