#include "los.h"

LOS::LOS(double ld, double ex_d, double ex_v,
double i, double r,
PID& y_pid) 
: R(ld), expc_dist(ex_d), expc_vel(ex_v), 
ki(i), kr(r), 
yaw_pid(y_pid)
{}

void LOS::yaw_ctrl(double yaw)
{
     double yaw_err = yaw;
        if (yaw_err > M_PI)
        {
            yaw_err -= M_PI;
        }
        else if (yaw_err < -M_PI)
        {
            yaw_err += M_PI;
        }

        double r_cmd = yaw_pid.pid_output(yaw_err);

         
            if (r_cmd > 5)
            {
                r_cmd = 5;
            }
            else if (r_cmd < -5)
            {
                r_cmd = -5;
            }
        

        uvr_cmd[2] = r_cmd;
}

void LOS::update(double yaw, double d)
{
    double phi = yaw + M_PI*0.5;
    double d_err = d - expc_dist;
    int_delta += d_err;
    double x = phi - atan(kr*(d_err+ki*int_delta)/R);
    uvr_cmd[0] = expc_vel * cos(x - yaw);
    uvr_cmd[1] = expc_vel * sin(x - yaw);
    yaw_ctrl(yaw); 
}

std::array<double, 3> LOS::cmd_get(double yaw, double d)
{
    update(yaw, d);
    return uvr_cmd;
}