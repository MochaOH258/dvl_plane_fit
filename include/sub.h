#ifndef SUB__H
#define SUB__H

#include <array>

#include "dvl_plane.h"
#include "pid.h"

class SubInterface{
    private:
        std::array<double, 4> dist_array{0.0, 0.0, 0.0, 0.0};
        std::array<bool, 4> beam_valid_array{0, 0, 0, 0};
        std::array<double, 3> ruv_cmd{0.0, 0.0, 0.0};
        double vel = 0.0;
        Plane& P;
        Beam& B1;
        Beam& B2;
        Beam& B3;
        Beam& B4;
        Controller& C;
    public:
        SubInterface(Plane &Pl, Beam& b1, Beam& b2, Beam& b3, Beam& b4, Controller& c);
        double update_data(std::array<double, 4> d_array, std::array<bool, 4> valid_array, double v);
        double update_cmd(std::array<double, 3> cmd);
        std::array<double, 3> get_cmd(std::array<double, 4> d_array, std::array<bool, 4> valid_array, double v);

};

#endif