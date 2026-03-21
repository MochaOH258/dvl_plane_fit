#include "main.h"

int main(void)
{
    using clock = std::chrono::steady_clock;

    double freq = 200.0;
    auto period = std::chrono::duration<double>(1.0 / freq);

    auto next_time = clock::now();
    auto last_time = clock::now();

    Beam B1(0.25*M_PI, 0.25*M_PI);
    Beam B2(0.25*M_PI, 0.25*M_PI);
    Beam B3(0.25*M_PI, 0.25*M_PI);
    Beam B4(0.25*M_PI, 0.25*M_PI);
    Plane P(B1, B2, B3, B4);

    auto now = clock::now();
    PID Yaw(1.0, 0.0, 0.0, std::chrono::duration<double>(now - last_time).count());
    PID Dist(1.0, 0.0, 0.0, std::chrono::duration<double>(now - last_time).count());
    PID Vel(1.0, 0.0, 0.0, std::chrono::duration<double>(now - last_time).count());
    Controller Ctrl(100.0, 0.0, 50.0, Yaw, Dist, Vel, P);

    while(true)
    {
        now = clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;

        /* code */

        next_time += std::chrono::duration_cast<clock::duration>(period);
        std::this_thread::sleep_until(next_time);
    }

    return 0;
}