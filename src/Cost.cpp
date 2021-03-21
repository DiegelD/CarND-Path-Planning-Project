#include "Cost.h"
#include "Car.h"

#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>



const float REACH_GOAL = pow(10, 6);
const float EFFICIENCY = pow(10, 5);
double max_s = 6945.554; // Given in the main path, out of the map

//   The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in 
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.
// ---> Trajectory have to be implemented into Car
//  ->>> map data have to be adjusted
float goal_distance_cost(Car &car,
                         const vector<vector<double> > &sensor_fusion,
                         map<string, double> &data )
{
  // Cost increases based on distance of intended lane (for planning a lane 
  //   change) and final lane of trajectory.
  // Cost of being out of goal lane also becomes larger as vehicle approaches 
  //   goal distance.

  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0) {
    cost = 1 - 2*exp(-(abs(car.goal_lane - data["intended_lane"]) / distance));  // Final lane delte
  } else {
    cost = 1;
  }
  return cost;
}
// Implement lane speed function
// car target_speed
float inefficiency_cost(Car &car,
                        const vector<vector<double> > &sensor_fusion,
                        map<string, double> &data)
{
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane.
  float proposed_speed_intended = lane_speed(sensor_fusion, data["intended_lane"]);
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = car.speed;
  }

 /* No Final Lane, hence the logic is just looking for the lane nighbors 
  float proposed_speed_final = lane_speed(sensor_fusion, car.final_lane);
  if (proposed_speed_final < 0) {
    proposed_speed_final = car.speed;
  }
*/

  float cost = ( car.speed - proposed_speed_intended ) / car.speed;
 
  return cost;
}


float lane_speed(const vector<vector<double> > & sensor_fusion, int lane)
{
  // All non ego vehicles in a lane have the same speed, so to get the speed 
  //   limit for a lane, we can just find one vehicle in that lane.
  bool car_found = false;
  for (int i = 0; i < sensor_fusion.size(); i++)
  {
      float d = sensor_fusion[i][6];
      car_found = false;

      if (d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2))
      {
        car_found = true;
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double speed = sqrt(vx * vx * vy * vy);
        return speed;
        }
  }
  // Found no vehicle in the lane
  if (car_found == false)
  {
   return -1.0;
  }
}

float calculate_cost(Car &car, const vector<vector<double> > &sensor_fusion )
{
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, double> data = get_helper_data(car);

  float cost = 0.0;

  vector<std::function<float( Car &, const vector<vector<double> > &,
   map<string, double> & )> > cf_list = {goal_distance_cost, inefficiency_cost}; //cf_list = {goal_distance_cost, inefficiency_cost};

  vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
    
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i] * cf_list[i](car, sensor_fusion, data);
    cost += new_cost;
  }

  return cost;
}

map<string, double> get_helper_data(Car &car)
{
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or 
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory. ---> Not used in this project
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help 
  //   differentiate between planning and executing a lane change in the 
  //   cost functions.
  map<string, double> data;
  double intended_lane;
  // Check und vergleiche den State status mit orginal 
  if (car.current_state.compare("PLCL") == 0)
  {
    intended_lane = car.lane - 1;
  }
  else if (car.current_state.compare("PLCR") == 0)
  {
    intended_lane = car.lane + 1;
  }
  else
  {
    intended_lane = car.lane;
  }

  float distance_to_goal = max_s - car.s;
  
  data["intended_lane"] = intended_lane;
  //trajectory_data["final_lane"] = final_lane;
  data["distance_to_goal"] = distance_to_goal;
    
  return data;
}