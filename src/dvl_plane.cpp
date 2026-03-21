#include <dvl_plane.h>

/* 
    
   
    param: valid                 波束有效标志位
    param: distance              斜距
    param: gamma, beta           安装角
    param: cos_gamma, sin_gamma
    param: cos_beta, sin_beta
    param: vec             波束向量，描述其分量大小与方向
*/
class Beam{
    private:
    bool valid;
    double distance;
    double gamma, beta;
    double cos_gamma, sin_gamma;
    double cos_beta, sin_beta;
    Eigen::Vector3d vec;

    void beam_vector_cal(void)
    {
        vec << distance*cos_gamma, distance*sin_gamma*cos_beta, distance*sin_gamma*sin_beta;
    }

    public:
    Beam(
        double g, double b
        ) 
        : valid(false), distance(0.0), gamma(g), beta(b){
            cos_gamma = cos(gamma);
            sin_gamma = sin(gamma);
            cos_beta = cos(beta);
            sin_beta = sin(beta);

        };
    
        /* 
            波束数据更新函数
            -从左到右应该传入：
            1. 斜距
            2. 波束有效标志位
        */
    void beam_data_update(double d, bool v)
    {
        if (v = true)
        {
            distance = d;
            beam_vector_cal();
        }
        
        valid = v;
    }

    bool valid_get(void) const
    {
        /* 
            有效位getter函数
            --返回值为波束有效位
        */
        return valid;
    }

    /* 
        
    */
    const Eigen::Vector3d& vector_get(void) const {return vec;}
    
};

/* 
    
  
*/
class Plane{
    private:
    bool valid;
    int beam_count;
    double d;
    double residual;
    Beam& B1;
    Beam& B2;
    Beam& B3;
    Beam& B4;
    
    Eigen::Vector3d n;
    
   /* 
        平面拟合实现

        对于有效波束数<3，将跳过计算并将有效位置零

        对于有效波束数=3，将使用三点直接拟合平面

        对于有效波束数>3，即为4，将使用协方差解最小二乘拟合平面并计算残差

        >使用eigen库进行计算
   */

    bool plane_cal(void)
    {
        Beam* beams[4] = {&B1, &B2, &B3, &B4};
        beam_count = int(B1.valid_get()) + int(B2.valid_get()) + int(B3.valid_get()) + int(B4.valid_get());
        if (beam_count < 3) 
        {
            valid = 0;
        }
        else if (beam_count == 3)
        {
            Eigen::Vector3d selected[3];
            int index=0;
            for (int i=0;i<4;i++)
            {
                if (beams[i]->valid_get())
                {
                    selected[index++] = beams[i]->vector_get();
                }
            }
            Eigen::Vector3d v1,v2;
            v1 = selected[1]-selected[0];
            v2 = selected[2]-selected[0];
            n = v1.cross(v2);
            n.normalize();
            /* 
                a(x-x0)+b(y-y0)+c(z-z0)=0;
                ax+by+cz+d=0;
                d=-ax0-by0-cz0;
            */
            if (n(0)<0)
            {
                n = -n;
            }
            d = -n.dot(selected[2]);
            

            valid = 1;
            residual=0;
        }
        else
        {
            
            
            Eigen::Vector3d p1, p2, p3, p4;
            
            p1 = beams[0]->vector_get();
            p2 = beams[1]->vector_get();
            p3 = beams[2]->vector_get();
            p4 = beams[3]->vector_get();
            
            Eigen::Vector3d centroid = (p1 + p2 + p3 + p4) / 4.0;

            Eigen::Matrix3d C = Eigen::Matrix3d::Zero();

            Eigen::Vector3d q1 = p1 - centroid;
            Eigen::Vector3d q2 = p2 - centroid;
            Eigen::Vector3d q3 = p3 - centroid;
            Eigen::Vector3d q4 = p4 - centroid;

            C += q1 * q1.transpose();
            C += q2 * q2.transpose();
            C += q3 * q3.transpose();
            C += q4 * q4.transpose();

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);

            n = es.eigenvectors().col(0);
            n.normalize();

            if (n(0)<0) {
                n = -n;
                }
            d = -n.dot(centroid);
           
            double e1 = n.dot(p1) + d; 
            double e2 = n.dot(p2) + d; 
            double e3 = n.dot(p3) + d; 
            double e4 = n.dot(p4) + d; 
            residual = std::sqrt((e1*e1 + e2*e2 + e3*e3 + e4*e4) / 4.0);
            /* 
                if (residual>?)
                {
                    valid = 0;
                }
                    //残差过大处理
            */

            valid = 1;
        }
        return valid;
    }

    public:
    Plane(Beam& Beam1, Beam& Beam2, Beam& Beam3, Beam& Beam4)
    : valid(false), beam_count(0),
      d(0.0), residual(0.0),
      B1(Beam1), B2(Beam2), B3(Beam3), B4(Beam4)
    {
        n = Eigen::Vector3d::Zero();
    }
    
    bool update(void)
    {
        /* 
            调用波束数据进行平面拟合，返回此次拟合出平面的有效位
            未对波束信息是否全部更新进行检查，需要在更新波束数据后调用
        */
        return plane_cal();
    }

    bool valid_get(void) const
    {
        /* 
            有效位getter函数
            --返回值为波束有效位
        */
        return valid;
    }

    const Eigen::Vector3d& vector_get(void) const {
        /* 
            返回ROV到平面的法向量
            向量x轴分量方向为x轴正方向
        */
        return n;
    }

    double d_get(void) const
    {
        return d;
    }

    double horizon_angle_get(void) const
    {
        return atan2(n(0), n(1));
    }

};
