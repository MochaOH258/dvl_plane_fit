#include <pid.h>
/* 
    todo
    修正本文件逻辑，包括但应该不止：
    surge_pid逻辑
    上升控制
    参数名称统一与优化
    头文件修正
*/

class Controller{
    private:
    //期望的水平夹角
    double expc_disdance;
    //期望巡检速度
    double expc_angle;
    double expc_v_sway;

    Yaw_PID& PID_Yaw;
    Surge_Pid& PID_Surge;
    Sway_Pid& PID_Sway;


    public:
    Controller(double ex_d, double ex_an, double ex_v,
    Yaw_PID& PID_Y, Surge_Pid& PID_Su, Sway_Pid& PID_Sw
    ) :
    expc_disdance(ex_d), expc_angle(ex_an), expc_v_sway(ex_v), 
    PID_Yaw(PID_Y), PID_Surge(PID_Su), PID_Sway(PID_Sw)
    {}

    double yaw_control_get(double y) const
    {
        return PID_Yaw.yaw_control(y, expc_angle);
    }

    double surge_control_get(double distance) const
    {
        return PID_Surge.surge_control(distance, expc_disdance);
    }

    double sway_control_get(double v_s) const
    {
        return PID_Sway.sway_control(v_s, expc_v_sway);
    }
};

class PID{
    protected:
    double kp;
    double ki;
    double kd;

    double err;
    double pre_err;
    double int_err;

     double pid_cal(double error)
        {
            int_err += error;
            /* 
                if (int_err>?)
                {
                    int_err = ?;
                }
                else if (int_err<?)
                {
                    int_err = ?;
                }
            */
           double delta = error - pre_err;
           pre_err = error;
           err = error;

           return kp*error + ki*int_err + kd*delta;
        }

    public:
    PID(double p, double i, double d): kp(p), ki(i), kd(d)
    {

    }
    void set_pid(double p, double i, double d)
    {
        kp = p;
        ki = i;
        kd = d;
    }

};

class Yaw_PID: public PID{
    private:
       

    public:
    Yaw_PID(double p, double i, double d) : PID(p, i, d){}
    double yaw_control(double yaw, double expc_yaw)
    {
        return pid_cal(yaw - expc_yaw);
    }
};

class Surge_Pid: public PID{
    private:

    public:
     Surge_Pid(double p, double i, double d) : PID(p, i, d){}
     double surge_control(double d, double expc_d)
     {
        return pid_cal(d-expc_d);
     }
};

class Sway_Pid: public PID{
    private:

    public:
    Sway_Pid(double p, double i, double d) : PID(p, i, d){}
    double sway_control(double v, double expc_v)
    {
        return pid_cal(v-expc_v);
    }
};
