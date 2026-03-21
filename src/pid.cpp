#include <pid.h>
/* 

*/

class Controller{
    private:
    double expc_distance;
    double expc_yaw_angle;
    double expc_v_sway;

    double yaw_error;

    //cmd[0] = r_cmd
    //cmd[1] = u_cmd
    //cmd[2] = v_cmd
    double cmd[3] = {0.0, 0.0, 0.0};

    PID& YawPID;
    PID& DistancePID;
    PID& VPID;

    Plane& MyPlane;

    double yaw_crtl(double yaw) 
    {
        double yaw_err = expc_yaw_angle - yaw;
        if (yaw_err > 180)
        {
            yaw_err -= 360.0;
        }
        else if (yaw_err < -180)
        {
            yaw_err += 360.0;
        }

        yaw_error = yaw_err;

        double r_cmd = YawPID.pid_output(yaw_err);

        /* 
            if (r > ?)
            {
                r = ?;
            }
            else if (r < ?)
            {
                r = ?;
            }
        */

        cmd[0] = r_cmd;
        return r_cmd;
    }

    double dist_ctrl(double d)
    {
        double dist_err = expc_distance - dist_err;

        double u_cmd = DistancePID.pid_output(dist_err);

        /* 限幅, 考虑yaw_error */
        cmd[1] = u_cmd;
        return u_cmd;
    }

    double v_ctrl(double v)
    {
        double v_err = expc_v_sway - v;

        double v_cmd = VPID.pid_output(v_err);
        /* 限幅 */
        cmd[2] = v_cmd;
        return v_cmd;
        
    }

    public:
    Controller(double expc_d, double expc_yaw, double expc_v, 
        PID& Yaw, PID& Distance, PID& V, 
        Plane& P
    ) :
    expc_distance(expc_d), expc_yaw_angle(expc_yaw), expc_v_sway(expc_v),
    YawPID(Yaw), DistancePID(Distance), VPID(V), 
    MyPlane(P), 
    yaw_error(0.0)
    {}

    void set_expc_distance(double dis)
    {
        expc_distance = dis;
    }

    void set_expc_yaw(double y)
    {
        expc_yaw_angle = y;
    }

    void set_expc_v_sway(double v)
    {
        expc_v_sway = v;
    }

    const double* cmd_get(double v)
    {
        if (MyPlane.valid_get())
        {
            yaw_crtl(MyPlane.horizon_angle_get());
            dist_ctrl(MyPlane.d_get());
        }
        v_ctrl(v);
        return cmd;
    }

    
};

class PID{
    private:
    double kp;
    double ki;
    double kd;

    double pre_err;
    double int_err;

     double pid_cal(double err)
        {
            int_err += err;
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
           double delta = err - pre_err;
           pre_err = err;

           return kp*err + ki*int_err + kd*delta;
        }

    public:
    PID(double p, double i, double d): kp(p), ki(i), kd(d), pre_err(0.0), int_err(0.0) {}
    void set_pid(double p, double i, double d)
    {
        kp = p;
        ki = i;
        kd = d;
    }
    double pid_output(double error)
    {
        return pid_cal(error);
    }

};
