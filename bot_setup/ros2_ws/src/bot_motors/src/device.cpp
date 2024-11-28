#include "bot_motors/device.hpp"

namespace device
{
    int Device::deviceCount = 0;
    int Device::gpioHandle = -1;

    Device::Device(unsigned int device_GPIO_pin, const std::string& deviceName) 
        : deviceName_(deviceName), device_GPIO_pin_(device_GPIO_pin)
    {
        if (deviceCount == 0)
        {
            //Locally connect to pigpio daemon (pigpiod)
            gpioHandle = pigpio_start(NULL,NULL);
            if (gpioHandle < 0)
            {
                throw std::runtime_error(fmt::format("Failed to connect to pigpiod\n"));
            }
        }
        deviceCount++;
    }

    Device::~Device()
    {
        deviceCount--;
        //fmt::print("Closing {}'s GPIO pin {}\n", deviceName_, device_GPIO_pin_);

        if (deviceCount == 0)
        {
            pigpio_stop(gpioHandle);
            fmt::print("Closing pigpiod daemon.\n");
        }
    }

    int Device::getGPIOHandle() const
    {
        return gpioHandle;
    }
}
