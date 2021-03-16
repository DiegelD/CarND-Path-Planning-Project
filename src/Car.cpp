

#include "Car.h"
#include <iostream>

using std::string;
using std::vector;
using namespace std;

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
    double distance2goal = 1;
    bool too_close = false;

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