#ifndef DVL_PLANE__H
#define DVL_PLANE__H

#include <cmath>

#include <Eigen/Dense>

class Beam{
    private:
    bool valid;
    double distance;
    double gamma, beta;
    double cos_gamma, sin_gamma;
    double cos_beta, sin_beta;
    Eigen::Vector3d vec;
    void beam_vector_cal(void);

    public:
    Beam(double g, double b);
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
    Beam& B1;
    Beam& B2;
    Beam& B3;
    Beam& B4;
    Eigen::Vector3d n;
    bool plane_cal(void);

    public:
    Plane(Beam& Beam1, Beam& Beam2, Beam& Beam3, Beam& Beam4);
    bool update(void);
    bool valid_get(void) const;
    const Eigen::Vector3d& vector_get(void) const;
    double d_get(void) const;
    double horizon_angle_get(void) const;
};

#endif