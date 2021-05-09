#include "Cost.h"
#include "Car.h"

#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>

using std::string;
using std::vector;
using namespace std;

const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);
double max_s = 6945.554; // Given in the main path, out of the map

//   The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in 
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.
// ---> Trajectory have to be implemented into Car
//  ->>> map data have to be adjusted

float goal_distance_cost(  const Car &car,
                           const vector<Car> &trajectory,
                           vector<Car> &predictions,
                           map<string, float> &data)
{
  // Cost increases based on distance of intended lane (for planning a lane
  //   change) and final lane of trajectory.
  // Cost of being out of goal lane also becomes larger as vehicle approaches
  //   goal distance.
  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0)
  {
    cost = 1 - 2 * exp(-(abs(2.0 * car.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
  }
  else
  {
    cost = 1;
  }
}

  // Implement lane speed function
  // car target_speed
float inefficiency_cost(  const Car &car,
                          const vector<Car> &trajectory,
                          const vector<Car> &predictions,
                          map<string, float> &data) 
{
  cout << "cc 3" << endl;
  // Cost becomes higher for trajectories with intended lane and final lane
  //  that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane.
  float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
  if (proposed_speed_intended < 0)
  {
    proposed_speed_intended = car.target_speed;
  }

  float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
  if (proposed_speed_final < 0)
  {
    proposed_speed_final = car.target_speed;
  }

  float cost = (2.0 * car.target_speed - proposed_speed_intended - proposed_speed_final) / car.target_speed;

  return cost;
};


float lane_speed( vector<Car> &predictions, int lane) 
{
  // All non ego vehicles in a lane have the same speed, so to get the speed 
  //   limit for a lane, we can just find one vehicle in that lane.
  for (int i = 0; i < predictions.size(); i++)
  {
    Car vehicle = predictions[i];
    if (vehicle.lane == lane)
    {
      return vehicle.v;
    }
  }
   return -1.0;
}

float calculate_cost(const Car &car,
                     const vector<Car> &predictions,
                     const vector<Car> &trajectory)
{
  cout << "cc 1" << endl;
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, float> trajectory_data = get_helper_data(car, trajectory, predictions);

  float cost = 0.0;

  vector<std::function<float( const Car &,
                              const vector<Car> &,
                              const vector<Car> &,
                              map<string, float> &)
                             >>  cf_list = {goal_distance_cost, inefficiency_cost}; //cf_list = {goal_distance_cost, inefficiency_cost};

  vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i] * cf_list[i](car, trajectory, predictions,
                                                 trajectory_data);
    cost += new_cost;
  }

  return cost;
}

map<string, float> get_helper_data(const Car &car,
                                    const vector<Car> &trajectory,
                                    const vector<Car> &predictions)
{
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or 
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory. ---> Not used in this project
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help 
  //   differentiate between planning and executing a lane change in the 
  //   cost functions.
  map<string, float> trajectory_data;
  Car trajectory_last = trajectory[1];
  float intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0)
  {
    intended_lane = trajectory_last.lane + 1;
  }
  else if (trajectory_last.state.compare("PLCR") == 0)
  {
    intended_lane = trajectory_last.lane - 1;
  }
  else
  {
    intended_lane = trajectory_last.lane;
  }

  float distance_to_goal = car.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;

  return trajectory_data;
}