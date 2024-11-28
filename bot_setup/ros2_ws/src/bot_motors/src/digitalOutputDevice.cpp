#include "bot_motors/digitalOutputDevice.hpp"

namespace outputDevice
{
    DigitalOutputDevice::~DigitalOutputDevice() 
    {
        off();
    }

    DigitalOutputDevice::DigitalOutputDevice(unsigned int gpioPin, 
        const std::string& digitalDeviceName) : 
            Device(gpioPin, digitalDeviceName) 
    {
        this -> digitalOutputPin_ = gpioPin;
        set_mode(getGPIOHandle(), gpioPin, PI_OUTPUT);
        off();
    }

    void DigitalOutputDevice::on()
    {
        gpio_write(getGPIOHandle(), digitalOutputPin_, 1);
    }

    void DigitalOutputDevice::off()
    {
        gpio_write(getGPIOHandle(), digitalOutputPin_, 0);
    }
}
