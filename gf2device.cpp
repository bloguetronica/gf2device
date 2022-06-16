/* GF2 device class - Version 0.1.0
   Requires CP2130 class version 1.1.0 or later
   Copyright (c) 2022 Samuel Lourenço

   This library is free software: you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published by
   the Free Software Foundation, either version 3 of the License, or (at your
   option) any later version.

   This library is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this library.  If not, see <https://www.gnu.org/licenses/>.


   Please feel free to contact me via e-mail: samuel.fmlourenco@gmail.com */


// Includes
#include <sstream>
#include <unistd.h>
#include <vector>
#include "gf2device.h"

// Definitions
const uint8_t EPOUT = 0x01;  // Address of endpoint assuming the OUT direction

GF2Device::GF2Device() :
    cp2130_()
{
}

// Diagnostic function used to verify if the device has been disconnected
bool GF2Device::disconnected() const
{
    return cp2130_.disconnected();
}

// Checks if the device is open
bool GF2Device::isOpen() const
{
    return cp2130_.isOpen();
}

// Closes the device safely, if open
void GF2Device::close()
{
    cp2130_.close();
}

// Returns the silicon version of the CP2130 bridge
CP2130::SiliconVersion GF2Device::getCP2130SiliconVersion(int &errcnt, std::string &errstr)
{
    return cp2130_.getSiliconVersion(errcnt, errstr);
}

// Returns the hardware revision of the device
std::string GF2Device::getHardwareRevision(int &errcnt, std::string &errstr)
{
    return hardwareRevision(getUSBConfig(errcnt, errstr));
}

// Gets the manufacturer descriptor from the device
std::u16string GF2Device::getManufacturerDesc(int &errcnt, std::string &errstr)
{
    return cp2130_.getManufacturerDesc(errcnt, errstr);
}

// Gets the product descriptor from the device
std::u16string GF2Device::getProductDesc(int &errcnt, std::string &errstr)
{
    return cp2130_.getProductDesc(errcnt, errstr);
}

// Gets the serial descriptor from the device
std::u16string GF2Device::getSerialDesc(int &errcnt, std::string &errstr)
{
    return cp2130_.getSerialDesc(errcnt, errstr);
}

// Gets the USB configuration of the device
CP2130::USBConfig GF2Device::getUSBConfig(int &errcnt, std::string &errstr)
{
    return cp2130_.getUSBConfig(errcnt, errstr);
}

// Opens a device and assigns its handle
int GF2Device::open(const std::string &serial)
{
    return cp2130_.open(VID, PID, serial);
}

// Issues a reset to the CP2130, which in effect resets the entire device
void GF2Device::reset(int &errcnt, std::string &errstr)
{
    cp2130_.reset(errcnt, errstr);
}

// Helper function that returns the hardware revision from a given USB configuration
std::string GF2Device::hardwareRevision(const CP2130::USBConfig &config)
{
    std::string revision;
    if (config.majrel > 1 && config.majrel <= 27) {
        revision += static_cast<char>(config.majrel + 'A' - 2);  // Append major revision letter (a major release number value of 2 corresponds to the letter "A" and so on)
    }
    if (config.majrel == 1 || config.minrel != 0) {
        std::ostringstream stream;
        stream << static_cast<int>(config.minrel);
        revision += stream.str();  // Append minor revision number
    }
    return revision;
}

// Helper function to list devices
std::list<std::string> GF2Device::listDevices(int &errcnt, std::string &errstr)
{
    return CP2130::listDevices(VID, PID, errcnt, errstr);
}
