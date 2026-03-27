#include "sub.h"

SubInterface::SubInterface(Plane &Pl, Beam& b1, Beam& b2, Beam& b3, Beam& b4, Controller& c) : P(Pl), B1(b1), B2(b2), B3(b3), B4(b4), C(c) {}

void SubInterface::update_data(std::array<double, 4> d_array, std::array<bool, 4> valid_array, double v)
        {
            dist_array = d_array;
            beam_valid_array = valid_array;
            vel = v;
        }

void SubInterface::update_cmd(std::array<double, 3> cmd)
{
    ruv_cmd = cmd;
}
std::array<double, 3> SubInterface::get_cmd(std::array<double, 4> d_array, std::array<bool, 4> valid_array, double v)
{
    update_data(d_array, valid_array, v);
    B1.beam_data_update(dist_array[0], beam_valid_array[0]);
    B2.beam_data_update(dist_array[1], beam_valid_array[1]);
    B3.beam_data_update(dist_array[2], beam_valid_array[2]);
    B4.beam_data_update(dist_array[3], beam_valid_array[3]);
    P.update();
    ruv_cmd = C.cmd_get(vel);
    return ruv_cmd;
}
