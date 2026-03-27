#ifndef LOS__H
#define LOS__H

#include <array>
#include <cmath>

#include "pid.h"

class LOS{
    private:
        double R;
        double int_delta = 0.0;

        double ki = 1.0;
        double kr = 1.0;
        
        double expc_dist;
        double expc_vel;

        PID& yaw_pid;

        std::array<double, 3> uvr_cmd{0.0, 0.0, 0.0};

        void update(double yaw, double d);
        void yaw_ctrl(double yaw);
    public:
    LOS(double ld, double ex_d, double ex_v, double i, double r, PID& y_pid);

    std::array<double, 3> cmd_get(double yaw, double d);
    

};

#endif