#include "bot_motors/motorDriver.hpp"

namespace motorDriver
{
    MotorDriver::MotorDriver(
        unsigned int input_1_pin, unsigned int input_2_pin,
        unsigned int enable_PWM_pin, std::string motorName):
            Device(enable_PWM_pin, motorName + "_enablePin"),
            input_1_pin_(input_1_pin, motorName + "_inputPin1"),
            input_2_pin_(input_2_pin, motorName + "_inputPin2"),
            motor_effort_percent_(0.0)
    {
        this -> enablePin_ = enable_PWM_pin;
        setDirection(motorDriver::MotorDirection::Forward);
    }

    MotorDriver::~MotorDriver()
    {
        setEffortPercent(0.0);
    }

    MotorDirection MotorDriver::getDirection() const
    {
        return motor_direction_;
    }

    void MotorDriver::setDirection(MotorDirection direction)
    {
        motor_direction_ = direction;
        if (direction == MotorDirection::Forward) 
        {
            input_1_pin_.on();
            input_2_pin_.off();
        }
        else
        {
            input_1_pin_.off();
            input_2_pin_.on();
        }
    }

    double MotorDriver::getEffortPercent() const
    {
        return motor_effort_percent_;
    }

    void MotorDriver::setEffortPercent(double new_percent)
    {

        if ((new_percent < 0.0) || (new_percent > 100.0))
        {
            throw std::runtime_error(
                fmt::format("Effort value {} is invalid. Valid range [0.0, 100.0]", 
                new_percent)); 
        }

        motor_effort_percent_ = new_percent / 100.0;

        auto int_percent = static_cast<uint8_t>(255.0 * motor_effort_percent_);
        auto result = set_PWM_dutycycle(getGPIOHandle(), enablePin_, int_percent);
        if (result == 0)
        {
            return;
        }

        switch (result)
        {       
            case PI_BAD_USER_GPIO:
            {
            throw std::runtime_error(fmt::format("{} is a bad GPIO pin", enablePin_));
            }

            case PI_BAD_DUTYCYCLE:
            {
                throw std::runtime_error(
                    fmt::format("Bad duty cycle specified for gpio pin {}", enablePin_));
            }

            default:
            {
                throw std::runtime_error(
                    fmt::format("Unexpected error encountered when setting mode specified for gpio pin {} (Error = {})", 
                        enablePin_, result));
            }
    }
    }
}
