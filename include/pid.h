#ifndef PID__H
#define PID__H

#include <Eigen/Dense>
#include <dvl_plane.h>



class PID{
    protected:
    double kp;
    double ki;
    double kd;
    double err;
    double pre_err;
    double int_err;
    double pid_cal(double error);

    public:
    PID(double p, double i, double d);
    void set_pid(double p, double i, double d);

};

class Yaw_PID: public PID{
    private:
       
    public:
    Yaw_PID(double p, double i, double d);
    double yaw_control(double yaw, double expc_yaw);
};

class Surge_Pid: public PID{
    private:

    public:
     Surge_Pid(double p, double i, double d);
     double surge_control(double d, double expc_d);
};

class Sway_Pid: public PID{
    private:

    public:
    Sway_Pid(double p, double i, double d);
    double sway_control(double v, double expc_v);
};


#endif