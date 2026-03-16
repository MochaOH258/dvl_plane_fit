#include <cmath>

#include <Eigen/Dense>
#include <Eigen/SVD>

/* 
    定义了Beam类，用于储存DVL波束的：
    //测距得到的斜距和有效位
    //安装角度信息和角度的三角函数值
    //计算出的在DVL设备坐标系下的波束向量数据

    同时对外暴露：
    //更新测量得到的斜距和有效位
    //有效位getter函数
    //波束向量数组指针getter函数

    在private属性中还包括基于斜距和安装角的向量计算实现
   
    param: valid                 波束有效标志位
    param: distance              斜距
    param: gamma, beta           安装角
    param: cos_gamma, sin_gamma
    param: cos_beta, sin_beta
    param: vector[3]             波束向量，描述其分量大小与方向
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
        valid = v;
        distance = d;
        beam_vector_cal();
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
    定义了Plane类
    用于储存由dvl波束测距点拟合出的平面数据，还存储了某次计算数据的有效位和使用的波束数
    对于四点进行平面拟合，还存储了计算出的残差
*/
class Plane{
    private:
    bool valid;
    int beam_count;
    double a,b,c,d;
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

        对于有效波束数>3，即为4，将使用svd解最小二乘拟合平面并计算残差
        >>使用eigen库实现这部分计算
   */

    int plane_cal(void)
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
            n.normalized();
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

            Eigen::Matrix<double, 4, 3> X;
            X.row(0) = (p1 - centroid).transpose();
            X.row(1) = (p2 - centroid).transpose();
            X.row(2) = (p3 - centroid).transpose();
            X.row(3) = (p4 - centroid).transpose();

            Eigen::JacobiSVD<Eigen::Matrix<double, 4, 3>> svd(
                X, Eigen::ComputeFullV
                );

            n = svd.matrixV().col(2);
            n.normalize();

            if (n.dot(centroid) > 0) {
                n = -n;
                }
            double d_1 = -n.dot(centroid);
            a = n.x();
            b = n.y();
            c = n.z();
            d = d_1;

            double e1 = n.dot(p1) + d_1;
            double e2 = n.dot(p2) + d_1;
            double e3 = n.dot(p3) + d_1;
            double e4 = n.dot(p4) + d_1;

            residual = std::sqrt((e1*e1 + e2*e2 + e3*e3 + e4*e4) / 4.0);

            valid = 1;
        }
        return valid;
    }

    public:
    Plane(Beam& Beam1, Beam& Beam2, Beam& Beam3, Beam& Beam4)
    : valid(false), beam_count(0),
      a(0.0), b(0.0), c(0.0), d(0.0), residual(0.0),
      B1(Beam1), B2(Beam2), B3(Beam3), B4(Beam4)
    {}
    


};
