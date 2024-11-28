#ifndef DEVICE
#define DEVICE

#include <pigpiod_if2.h>
#include <fmt/core.h>
#include <iostream>
#include <string>

namespace device
{
    class Device
    {
        public:
            Device(unsigned int device_GPIO_pin, const std::string& deviceName="");
            virtual ~Device();

        protected:
            int getGPIOHandle() const;
            unsigned int device_GPIO_pin_; 
            std::string deviceName_;

        private:
            //Number of GPIO pins in use when using pigpio library 
            static int deviceCount; 
            static int gpioHandle;
    };
}
#endif