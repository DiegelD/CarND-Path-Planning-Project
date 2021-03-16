#include "BehaviourPlanner.h"
#include "Car.h"

#include <algorithm>
#include <iostream>
#include <cmath>

using namespace std;

BehaviourPlanner::BehaviourPlanner()
{
    bool lane_free_one = false;
    bool lane_free_two = false;
    bool lane_free_three= false;
    bool close_object_CR_ = false;
    bool close_object_LL_ = false;
    bool close_object_LM_ = false;
    ;
    bool close_object_LR_ = false;
    ;
    //bool KeepLane = true;
    BP_State state = Keep_Lane;
}

BehaviourPlanner::~BehaviourPlanner() {}

void BehaviourPlanner::lane_check(vector<vector<double> > sensor_fusion, Car car, int prev_size)
{
    cout << "Entered Behaviour Planer lane check               " << endl;
    //find ref_v to use

    bool close_object_ = false; // Flag for detecting a closed object i a lane

    for (int i = 0; i < sensor_fusion.size(); i++)
    {
        // car is in my lane
        float d = sensor_fusion[i][6];
        if (d < (2 + 4 * car.lane + 2) && d > (2 + 4 * car.lane - 2))
        {
            cout << "Entered Same Lane check              " << endl;
            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx * vx * vy * vy);
            double check_car_s = sensor_fusion[i][5];

            check_car_s += ((double)prev_size * .02 * check_speed); // IF using previous can project s value out
            // check s value greater than mine and s gap
            cout << "check_car_s > car.s        " << check_car_s << " " << car.s << endl;
            if ((check_car_s > car.s) && ((check_car_s - car.s) < 30))
            {
                // Do some logic here, lower reference velocity so we dont crash into the car infront of us, coud also flag to tray to change lanes or use ACC
                //ref_vel = 29.5; // MPH
                car.too_close = true;
                close_object_ = true;

                cout << "Car too close -> True          " << car.too_close << endl;
                cout << "Cclose_object_  -> True          " << close_object_ << endl;
                // Start checking the lanes around us
                this->lane_check_around_(sensor_fusion, car, prev_size);
            }
            else
            {
                cout << "close_object        " << close_object_<< endl;
                if (close_object_ == false) // Just entering if no closed object in the lane detected
                {
                    
                    car.too_close = false;
                    cout << "Close Car detected       " << car.too_close << endl;
                }
            }
        }
    }
}
void BehaviourPlanner::lane_check_around_(vector<vector<double> > sensor_fusion, Car car, int prev_size)
{
    cout << "Entered Behaviour Planer lane check AROUND             " << endl;
    close_object_LL_ = false;
    close_object_LM_ = false;
    close_object_LR_ = false;

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
                lane_free_one = false;
                close_object_LL_ = true;
            }
            else
            {
                if (close_object_LL_ == false)
                {
                    lane_free_one = true;
                }
            }
            // Output
            cout << "lane_free_one                " << lane_free_one << " " << endl;
        }
        // Lane lane 1

        if (d < (2 + 4 * 2 + 2) && d > (2 + 4 * 2 - 2))
        {
            Car cars_left;
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
                lane_free_two = false;
                close_object_LM_ = true;
            }
            else
            {
                if (close_object_LM_ == false)
                {
                    lane_free_two = true;
                }
            }
            // Output
            cout << "lane_free_two                " << lane_free_two << " " << endl;
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
                lane_free_three = false;
                close_object_LR_ = true;
            }
            else
            {
                if (close_object_LR_ == false)
                {
                    lane_free_two = true;
                }
            }
            // Output
            cout << "lane_free_three                " << lane_free_three << " " << endl;
        }
    }
}


