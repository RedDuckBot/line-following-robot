#include "PID.hpp"

namespace pid_controller
{
    PID::PID(double kp, double kd, double ki, double time_interval)
    {
        this -> KP = kp;
        this -> KD = kd;
        this -> KI = ki;
        this -> integral_error = 0;
    }

    double PID::compute_adjustment(double current_error)
    {
        integral_error += time_interval * current_error;
        derivation_error = (current_error - prev_error) / time_interval;
        prev_error = current_error;

        return (KP * current_error) + (KI * integral_error) + (KD * derivation_error);
    }

    double PID::get_error() 
    {
        return prev_error;
    }
}