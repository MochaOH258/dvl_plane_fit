
#include "pid.h"
#include "dvl_plane.h"
#include "sub.h"

#include <iostream>
#include <chrono>
#include <thread>

int main(void)
{
    double t = 1/200;
    Beam B1(0.955316618, 0.785398163);
    Beam B2(0.955316618, -0.785398163);
    Beam B3(-0.955316618, 0.785398163);
    Beam B4(-0.955316618, -0.785398163);
    Plane P(B1, B2, B3, B4);
    PID Yaw(1.0, 0.0, 0.0, t);
    PID Dist(1.0, 0.0, 0.0, t);
    PID Vel(1.0, 0.0, 0.0, t);
    Controller Ctrl(100.0, 0.0, 50.0, Yaw, Dist, Vel, P);

    std::array<double, 4> dist{2.0, 2.1, 1.9, 2.0};
    std::array<bool, 4> valid{true, true, true, true};
    double v_sway = 0.0;

    

   

    return 0;
}