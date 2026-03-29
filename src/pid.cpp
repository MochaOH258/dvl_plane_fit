#include "pid.h"
/* 

*/

/* 
    todo
    完善拟合失效处理，现有逻辑如下：
    >连续10次拟合失败后，每次拟合失败减小对u和角速度的期望数值的修正系数（原1.0， -0.2每次，直到0），拟合成功恢复
    微分滤波
*/
    double Controller::yaw_ctrl(double yaw) 
    {
        
        double yaw_err = yaw - expc_yaw_angle;
        if (yaw_err > M_PI)
        {
            yaw_err -= M_PI;
        }
        else if (yaw_err < -M_PI)
        {
            yaw_err += M_PI;
        }

        yaw_error = yaw_err;

        double r_cmd = YawPID.pid_output(yaw_err);

        cmd[idx_r] = r_cmd;
        return r_cmd;
    }

    double Controller::dist_ctrl(double d)
    {
        double dist_err = d - expc_distance;

        double u_cmd = DistancePID.pid_output(dist_err);

        /* 限幅, 考虑yaw_error */
        if (std::abs(yaw_error) > 20)
        {
            u_cmd*=0.2;
        }
        
        cmd[idx_u] = u_cmd;
        return u_cmd;
    }

    double  Controller::v_ctrl(double v)
    {
        double v_err = expc_v_sway - v;

        double v_cmd = VPID.pid_output(v_err);
        /* 限幅 */
        if (std::abs(yaw_error) > 20)
        {
            v_cmd*=0.2;
        }
        
        cmd[idx_v] = v_cmd;
        return v_cmd;
        
    }

     Controller::Controller(double expc_d, double expc_yaw, double expc_v, 
        PID& Yaw, PID& Distance, PID& V, 
        Plane& P
    ) :
    expc_distance(expc_d), expc_yaw_angle(expc_yaw), expc_v_sway(expc_v),
    YawPID(Yaw), DistancePID(Distance), VPID(V), 
    MyPlane(P), 
    yaw_error(0.0)
    {}

    void  Controller::set_expc_distance(double dis)
    {
        expc_distance = dis;
    }

    void  Controller::set_expc_yaw(double y)
    {
        expc_yaw_angle = y;
    }

    void  Controller::set_expc_v_sway(double v)
    {
        expc_v_sway = v;
    }

    std::array<double, 3>  Controller::cmd_get(double v)
    {
        if (MyPlane.valid_get())
        {
            yaw_ctrl(MyPlane.horizon_angle_get());
            dist_ctrl(MyPlane.d_get());
            if (kr <= 0.8)
            {
                kr += 0.2;
            }
            else
            {
                kr = 1.0;
            }
            if (ku <= 0.8)
            {
                ku += 0.2;
            }
            else
            {
                ku = 1.0;
            }
            if (n != 10)
            {
                n = 10;
            }
        }
        else
        {
            if (n>=1)
            {
                n--;
            }
            if (n==0)
            {
                if (kr >= 0.1)
                {
                    kr-=0.1;
                }
                else
                {
                    kr = 0.0;
                }
                if (ku >= 0.1)
                {
                    ku-=0.1;
                }
                else
                {
                    ku = 0.0;
                }
            }
            
        }
        v_ctrl(v);
        std::array<double, 3> k_cmd{kr*cmd[idx_r], ku*cmd[idx_u], cmd[idx_v]}; 
        return k_cmd;
    }

    

/* 
    todo
    积分限幅
*/

    double PID::pid_cal(double err)
        {
            //temp
            //可能加微分滤波
            int_err += err*dt;
            
                if (int_err>10)
                {
                    int_err = 10;
                }
                else if (int_err<-10)
                {
                    int_err = -10;
                }
            
           double delta = (err - pre_err)/dt;
           pre_err = err;

           double output = kp*err + ki*int_err + kd*delta;

            if (output > max)
            {
                output = max;
            }
            else if (output < -max)
            {
                output = -max;
            }
            

           return output;
        }

    PID::PID(double p, double i, double d, double t, double m): kp(p), ki(i), kd(d), pre_err(0.0), int_err(0.0), dt(t), max(m){}

    void PID::set_pid(double p, double i, double d)
    {
        kp = p;
        ki = i;
        kd = d;
    }

    double PID::pid_output(double error)
    {
        return pid_cal(error);
    }

    void PID::set_dt(double t)
    {
        dt = t;
    }


