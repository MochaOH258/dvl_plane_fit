#ifndef DVL_PLANE__H
#define DVL_PLANE__H

#include <cmath>

#include <Eigen/Dense>

class Beam{
    private:
    /* 
    param: valid                 波束有效标志位
    param: distance              斜距
    param: gamma, beta           安装角
    param: cos_gamma, sin_gamma
    param: cos_beta, sin_beta
    param: vec             波束向量（根据斜距，安装角，
                            计算出为dvl为原点，
                            x轴正方向为前向，y轴正方向在水平面上垂直x轴向右，z轴正方向垂直x_y平面向上）
*/
    bool valid;
    double distance;
    double gamma, beta;
    double cos_gamma, sin_gamma;
    double cos_beta, sin_beta;
    Eigen::Vector3d vec;
    void beam_vector_cal(void);

    public:
    Beam(double g, double b);
    /* 
            波束数据更新函数
            -从左到右应该传入：
            1. 斜距
            2. 波束有效标志位
        */
    void beam_data_update(double d, bool v);
    bool valid_get(void) const;
    const Eigen::Vector3d& vector_get(void) const;
};

class Plane{
    private:
    bool valid;
    int beam_count;
    double d;
    double residual;
    double max_residual;
    Beam& B1;
    Beam& B2;
    Beam& B3;
    Beam& B4;
    Eigen::Vector3d n;
    /* 
        对于有效波束数<3，将跳过计算并将有效位置零；
        对于有效波束数=3，将使用三点直接拟合平面；对于有效波束数>3，
        即为4，将使用协方差解最小二乘拟合平面并计算残差
   */
    bool plane_cal(void);

    public:
    Plane(Beam& Beam1, Beam& Beam2, Beam& Beam3, Beam& Beam4, double max_r);
    /* 
            调用波束数据进行平面拟合，返回此次拟合出平面的有效位
            未对波束信息是否全部更新进行检查，需要在更新波束数据后调用
        */
    bool update(void);
    bool valid_get(void) const;
    const Eigen::Vector3d& vector_get(void) const;
    double d_get(void) const;
    
    /* 
        返回用弧度表示的水平夹角
    */
    double horizon_angle_get(void) const;
};

#endif