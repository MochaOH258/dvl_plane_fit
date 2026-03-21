#include <pid.h>
/* 

*/

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

    PID& YawPID;
    PID& DistancePID;
    PID& VPID;

    Plane& MyPlane;

    double yaw_ctrl(double yaw) 
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

        cmd[idx_r] = r_cmd;
        return r_cmd;
    }

    double dist_ctrl(double d)
    {
        double dist_err = expc_distance - d;

        double u_cmd = DistancePID.pid_output(dist_err);

        /* 限幅, 考虑yaw_error */
        cmd[idx_u] = u_cmd;
        return u_cmd;
    }

    double v_ctrl(double v)
    {
        double v_err = expc_v_sway - v;

        double v_cmd = VPID.pid_output(v_err);
        /* 限幅 */
        cmd[idx_v] = v_cmd;
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

    std::array<double, 3> cmd_get(double v)
    {
        if (MyPlane.valid_get())
        {
            yaw_ctrl(MyPlane.horizon_angle_get());
            dist_ctrl(MyPlane.d_get());
        }
        else
        {
            cmd[idx_r] = 0.0;
            cmd[idx_u] = 0.0;
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
