#ifndef OUTPUTDEVICE
#define OUTPUTDEVICE

#include <pigpio.h>
#include <fmt/core.h>
#include "device.hpp"

using device::Device;

namespace outputDevice
{
    class DigitalOutputDevice : public Device
    {
    	public:
            DigitalOutputDevice(unsigned int gpioPin, 
                const std::string& digitalDeviceName="");

            void on();
            void off();
            virtual ~DigitalOutputDevice();

        private:
            unsigned int digitalOutputPin_;
    };
}
#endif 
