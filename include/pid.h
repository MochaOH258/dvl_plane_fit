#ifndef PID__H
#define PID__H

#include <array>

#include "dvl_plane.h"

class PID{
    protected:
    double kp;
    double ki;
    double kd;

    double pre_err;
    double int_err;
    
    double pid_cal(double err);

    public:
    PID(double p, double i, double d);
    void set_pid(double p, double i, double d);
    double pid_output(double error);

};

class Controller{
    private:
    double expc_distance;
    double expc_yaw_angle;
    double expc_v_sway;

    double yaw_error;

    static constexpr int idx_r = 0;
    static constexpr int idx_u = 1;
    static constexpr int idx_v = 2;
    std::array<double,3> cmd{0.0, 0.0, 0.0};
    double kr = 1.0;
    double ku = 1.0;
    int n = 10;//temp

    PID& YawPID;
    PID& DistancePID;
    PID& VPID;

    Plane& MyPlane;

    double yaw_ctrl(double yaw);
    double dist_ctrl(double d);
    double v_ctrl(double v);

    public:
    Controller(double expc_d, double expc_yaw, double expc_v, 
        PID& Yaw, PID& Distance, PID& V, 
        Plane& P
    );
    void set_expc_distance(double dis);
    void set_expc_yaw(double y);
    void set_expc_v_sway(double v);
    std::array<double, 3> cmd_get(double v);
};

#endif