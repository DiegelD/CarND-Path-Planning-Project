#ifndef CAR_H
#define CAR_H


#include <string>
#include <vector>


class Car
{
public:
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
    int lane;
    int indended_lane;
    int final_lane;
    double distance2goal;
    bool too_close;

    Car();
    ~Car();

    //vector<string> sucessor_state();

    void update(double x_in, double y_in, double vx_in, double vy_in, double s_in, double d_in);
    void info();
};

#endif