#ifndef COST_H
#define COST_H

#include "Car.h"
#include <map>

using std::map;
using std::string;
using std::vector;


float calculate_cost(const Car &car,
                     const vector<Car> &predictions,
                     const vector<Car> &trajectory);

float goal_distance_cost(const Car &car,
                         const vector<Car> &trajectory,
                         const vector<Car> &predictions,
                         map<string, float> &data);

float inefficiency_cost(const Car &car,
                        const vector<Car> &trajectory,
                        const vector<Car> &predictions,
                        map<string, float> &data);

float lane_speed(const vector<Car> &predictions, int lane);

map<string, float> get_helper_data(const Car &car,
                                   const vector<Car> &trajectory,
                                   const vector<Car> &predictions);

#endif  // COST_H
