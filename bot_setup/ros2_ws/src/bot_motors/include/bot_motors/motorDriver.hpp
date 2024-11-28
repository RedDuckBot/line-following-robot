#ifndef MOTOR_DRIVER
#define MOTOR_DRIVER

#include "digitalOutputDevice.hpp"
#include "device.hpp"
#include <fmt/core.h>

using device::Device;
using outputDevice::DigitalOutputDevice;

namespace motorDriver
{
    enum MotorDirection
    {
        Forward,
        Backward
    };

    class MotorDriver : Device 
    {
        public:
            ///@brief Construct a MotorDriver object
            ///@param input_1_pin the GPIO number of pin connected to 'IN1' on motor controller
            ///@param input_2_pin ... connected to 'IN2' 
            ////@param enable_PWM_pin the GPIO pin connected to 'EN' pin on motor controller
            MotorDriver(
                unsigned int input_1_pin, unsigned int input_2_pin, 
                unsigned int enable_PWM_pin,
                std::string motorName);

            ~MotorDriver();

            ///@brief Get current direction of motor(s)
            ///@return MotorDirection enumeration
            MotorDirection getDirection() const;

            void setDirection(MotorDirection direction);

            ///@brief Get percentage that motor(s) are operating which is referred to
            ///       as the applied effort
            ///@return percentage as a double [0.0, 100.0]
            double getEffortPercent() const;

            ///@brief Set effort of motor(s) where 0% = stopping & 100% = max speed
            ///@param percent the new effort value
            void setEffortPercent(double percent);

            private:
                DigitalOutputDevice input_1_pin_;
                DigitalOutputDevice input_2_pin_;
                MotorDirection motor_direction_;
                unsigned int enablePin_;
                double motor_effort_percent_;
    };
}
#endif