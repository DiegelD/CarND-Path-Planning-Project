#ifndef COST_H
#define COST_H

#include "Car.h"
#include <map>

using std::map;
using std::string;
using std::vector;

float calculate_cost(Car &car,
                     const vector<vector<double> > &sensor_fusion);

float goal_distance_cost(Car &car,
                         const vector<vector<double> > &sensor_fusion,
                         map<string, double> &data);

float inefficiency_cost(Car &car,
                        const vector<vector<double> > &sensor_fusion,
                        map<string, double> &data);

float lane_speed(const vector<vector<double> > &sensor_fusion, int lane);

map<string, double> get_helper_data(Car &car);

#endif  // COST_H
