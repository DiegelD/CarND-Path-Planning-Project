#include "BehaviourPlanner.h"

#include <algorithm>
#include <iostream>
#include <cmath>

BehaviourPlanner::BehaviourPlanner()
{
    bool lane_free_one_ = false;
    bool lane_free_two_ = false;
    bool lane_free_three_ = false;
    bool KeepLane = "true";
}

BehaviourPlanner::~BehaviourPlanner(){}

void BehaviourPlanner::lane_check(auto sensor_fusion, Car &car, int prev_size)
{
    //find ref_v to use
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        // car is in my lane
        float d = sensor_fusion[i][6];
        if (d < (2 + 4 * car.lane + 2) && d > (2 + 4 * car.lane - 2))
        {
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx * vy * vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size * .02 * check_speed); // IF using previous can project s value out
            // check s value greater than mine and s gap
            if ((check_car_s > car.s) && ((check_car_s - car.s) < 30))
            {
                // Do some logic here, lower reference velocity so we dont crash into the car infront of us, coud also flag to tray to change lanes or use ACC
                //ref_vel = 29.5; // MPH
                bool KeepLane = false;
                // Start checking the lanes around us
                self.lane_check_around_(auto sensor_fusion, Car &car, int prev_size);
            }
            else{ 
                bool KeepLane = true;
            }
        }
    }
}
void BehaviourPlanner::lane_check_around_(auto sensor_fusion, Car &car, int prev_size)
{
    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        float d = sensor_fusion[i][6];
        // 1 lane check
    
            if (d < (2 + 4 * 1 + 2) && d > (2 + 4 * 1 - 2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx * vy * vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += ((double)prev_size * .02 * check_speed); // IF using previous can project s value out
                // check s value greater than mine and s gap
                if ((check_car_s > car.s) && ((check_car_s - car.s) < 30))
                {
                    // Do some logic here, lower reference velocity so we dont crash into the car infront of us, coud also flag to tray to change lanes or use ACC
                    //ref_vel = 29.5; // MPH
                    lane_free_one_ = false;
                }
                else
                {
                    lane_free_one_ = true;
                }
                // Output
                cout << "lane_free_one                " << lane_free_one_ << " " << endl;
            }
        // Lane lane 2
        
            if (d < (2 + 4 * 2 + 2) && d > (2 + 4 * 2 - 2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx * vy * vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += ((double)prev_size * .02 * check_speed); // IF using previous can project s value out
                // check s value greater than mine and s gap
                if ((check_car_s > car.s) && ((check_car_s - car.s) < 30))
                {
                    // Do some logic here, lower reference velocity so we dont crash into the car infront of us, coud also flag to tray to change lanes or use ACC
                    //ref_vel = 29.5; // MPH
                    lane_free_two_ = false;
                }
                else
                {
                    lane_free_two_ = true;
                }
                // Output
                cout << "lane_free_two                " << lane_free_two_ << " " << endl;
            }
        // Lane 3
            if (d < (2 + 4 * 3 + 2) && d > (2 + 4 * 3 - 2))
            {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx * vx * vy * vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += ((double)prev_size * .02 * check_speed); // IF using previous can project s value out
                // check s value greater than mine and s gap
                if ((check_car_s > car.s) && ((check_car_s - car.s) < 30))
                {
                    // Do some logic here, lower reference velocity so we dont crash into the car infront of us, coud also flag to tray to change lanes or use ACC
                    //ref_vel = 29.5; // MPH
                    lane_free_three_ = false;
        
                }
                else
                {
                    lane_free_three_ = true;
                }
                // Output
                cout << "lane_free_three                " << lane_free_three_ << " " << endl;
            }
    }
}

/*vector<Vehicle::State> get_sucessor_State(Vehicle &vehicle){

    vector<Vehicle::State> successor_states;
}


void update(Vehicle &vehicle);