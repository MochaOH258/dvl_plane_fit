#include "dvl_plane.h"



    void Beam::beam_vector_cal(void)
    {
        vec << distance*cos_gamma, distance*sin_gamma*cos_beta, distance*sin_gamma*sin_beta;
    }

    Beam::Beam(
        double g, double b
        ) 
        : valid(false), distance(0.0), gamma(g), beta(b){
            cos_gamma = cos(gamma);
            sin_gamma = sin(gamma);
            cos_beta = cos(beta);
            sin_beta = sin(beta);

        }
    
        
    void Beam::beam_data_update(double d, bool v)
    {
        if (v == true)
        {
            distance = d;
            beam_vector_cal();
        }
        
        valid = v;
    }

    bool Beam::valid_get(void) const
    {
        /* 
            有效位getter函数
            --返回值为波束有效位
        */
        return valid;
    }

    const Eigen::Vector3d& Beam::vector_get(void) const {return vec;}
    

    bool Plane::plane_cal(void)
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
             
            valid = 1;
            if (residual > 40)
            {
                valid = 0;
            }
                    //残差过大处理
                    //temp

            
        }
        return valid;
    }

    Plane::Plane(Beam& Beam1, Beam& Beam2, Beam& Beam3, Beam& Beam4)
    : valid(false), beam_count(0),
      d(0.0), residual(0.0),
      B1(Beam1), B2(Beam2), B3(Beam3), B4(Beam4)
    {
        n = Eigen::Vector3d::Zero();
    }
    
    bool Plane::update(void)
    {
        return plane_cal();
    }

    bool Plane::valid_get(void) const
    {
        return valid;
    }

    const Eigen::Vector3d& Plane::vector_get(void) const {
        /* 
            返回ROV到平面的法向量
            向量x轴分量方向为x轴正方向
        */
        return n;
    }

    double Plane::d_get(void) const
    {
        return std::abs(d);
    }

    double Plane::horizon_angle_get(void) const
    {
        return std::atan2(n(1), n(0));
    }
