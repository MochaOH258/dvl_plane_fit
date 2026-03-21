#ifndef PID__H
#define PID__H

#include <Eigen/Dense>
#include <dvl_plane.h>

class Controller{
    private:
    double expc_distance;
    double expc_yaw_angle;
    double expc_v_sway;

    double cmd[3];

    PID& YawPID;
    PID& DistancePID;
    PID& VPID;

    Plane& MyPlane;

    double yaw_crtl(double yaw) const;
    double dist_ctrl(double d) const;
    double v_ctrl(double v) const;

    public:
    Controller(double expc_d, double expc_yaw, double expc_v, 
        PID& Yaw, PID& Distance, PID& V
    );
    void set_expc_distance(double dis);
    void set_expc_yaw(double y);
    void set_expc_v_sway(double v);
    const double* cmd_get(double v);
};

class PID{
    protected:
    double kp;
    double ki;
    double kd;

    double pre_err;
    double int_err;

    public:
    PID(double p, double i, double d);
    void set_pid(double p, double i, double d);
    double pid_output(double error);

};


#endif