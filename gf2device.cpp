/* GF2 device class - Version 0.7.0
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
#include <cmath>
#include <sstream>
#include <unistd.h>
#include <vector>
#include "gf2device.h"

// Definitions
const uint8_t EPOUT = 0x01;   // Address of endpoint assuming the OUT direction
const uint8_t FREQ0 = 0x40;   // Mask for the FREQ0 register
const uint8_t FREQ1 = 0x80;   // Mask for the FREQ1 register
const uint8_t PHASE0 = 0xc0;  // Mask for the PHASE0 register
const uint8_t PHASE1 = 0xe0;  // Mask for the PHASE1 register

// Amplitude conversion constant
const uint AQUANTUM = 1023;  // Quantum related to the 10-bit resolution of the AD5310 DAC

// Frequency conversion constants
const uint FQUANTUM = 268435456;  // Quantum related to the 28-bit frequency resolution of the AD9834 waveform generator
const float MCLK = 80000;         // 80MHz clock

// Phase conversion constant
const uint PQUANTUM = 4096;  // Quantum related to the 12-bit phase resolution of the AD9834 waveform generator

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

// Sets the frequency, phase and amplitude of the generated signal to zero, and sets its waveform to sinusoidal
void GF2Device::clear(int &errcnt, std::string &errstr)
{
    cp2130_.setGPIO2(false, errcnt, errstr);  // If not already, set GPIO.2 to a logical low in preparation for reset (GPIO.2 corresponds to the RST signal and is connected to the RESET pin on the AD9834 waveform generator)
    cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
    std::vector<uint8_t> setupAD9834 = {
        0x22, 0x00  // B28 = 1, PIN/SW = 1, MODE = 0 (sinusoidal waveform)
    };
    cp2130_.spiWrite(setupAD9834, EPOUT, errcnt, errstr);  // Configure the AD9834 (channel 0) so that it acknowledges reset by pin
    usleep(100);  // Wait 100us, in order to prevent possible errors while setting GPIO.2 high (workaround)
    cp2130_.setGPIO2(true, errcnt, errstr);  // Set GPIO.2 to a logical high to enable reset
    std::vector<uint8_t> clearAD9834 = {
        FREQ0, 0x00, FREQ0, 0x00,  // FREQ0 register set to zero
        FREQ1, 0x00, FREQ1, 0x00,  // FREQ1 register set to zero
        PHASE0, 0x00,              // PHASE0 register set to zero
        PHASE1, 0x00               // PHASE1 register set to zero
    };
    cp2130_.spiWrite(clearAD9834, EPOUT, errcnt, errstr);  // Clear all of the AD9834 frequency and phase registers in order to set both generation parameters to zero
    usleep(100);  // Wait 100us, in order to prevent possible errors while switching the chip select (workaround)
    cp2130_.selectCS(1, errcnt, errstr);  // Enable the chip select corresponding to channel 1, and disable the one corresponding to channel 0 (the previously selected channel)
    std::vector<uint8_t> clearAD5310 = {
        0x00, 0x00  // AD5310 register set to zero
    };
    cp2130_.spiWrite(clearAD5310, EPOUT, errcnt, errstr);  // Clear the AD5310 register in order to set the amplitude to zero
    usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
    cp2130_.disableCS(1, errcnt, errstr);  // Disable the chip select corresponding to channel 1, which is the only one that is active to this point
    cp2130_.setGPIO3(false, errcnt, errstr);  // Set GPIO.3 to a logical low to enable the AD9834 internal DAC (GPIO.3 corresponds to the SLP signal and is connected to the SLEEP pin on the AD9834 waveform generator)
    cp2130_.setGPIO4(false, errcnt, errstr);  // Set GPIO.4 to a logical low, so that the FREQ0 register defines the frequency of the AD9834 (GPIO.4 corresponds to the FSEL signal and is connected to the FSELECT pin on the AD9834 waveform generator)
    cp2130_.setGPIO5(false, errcnt, errstr);  // Set GPIO.5 to a logical low, so that the PHASE0 register defines the phase of the AD9834 (GPIO.5 corresponds to the PSEL signal and is connected to the PSELECT pin on the AD9834 waveform generator)
    cp2130_.setGPIO6(false, errcnt, errstr);  // Set GPIO.6 to a logical low to enable the synchronous clock generation  (GPIO.6 corresponds to the !CMPEN signal and is connected to the SHDN pin on the TLV3501 comparator)
    cp2130_.setGPIO2(false, errcnt, errstr);  // Set GPIO.2 to a logical low to disable reset (the waveform generator is now enabled)
}

// Closes the device safely, if open
void GF2Device::close()
{
    cp2130_.close();
}

// Gets the status of the synchronous clock (enabled or disabled)
bool GF2Device::getClockStatus(int &errcnt, std::string &errstr)
{
    return !cp2130_.getGPIO6(errcnt, errstr);  // GPIO.6 corresponds to the !CMPEN signal and is connected to the SHDN pin on the TLV3501 comparator
}


// Returns the silicon version of the CP2130 bridge
CP2130::SiliconVersion GF2Device::getCP2130SiliconVersion(int &errcnt, std::string &errstr)
{
    return cp2130_.getSiliconVersion(errcnt, errstr);
}

// Gets the status of the DAC internal to the AD9834 waveform generator (enabled or disabled)
bool GF2Device::getDACStatus(int &errcnt, std::string &errstr)
{
    return !cp2130_.getGPIO3(errcnt, errstr);  // GPIO.3 corresponds to the SLP signal and is connected to the SLEEP pin on the AD9834 waveform generator
}

// Returns the current frequency selection
bool GF2Device::getFrequencySelection(int &errcnt, std::string &errstr)
{
    return cp2130_.getGPIO4(errcnt, errstr);  // GPIO.4 corresponds to the FSEL signal (FSELECT pin on the AD9834 waveform generator)
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

// Returns the current phase selection
bool GF2Device::getPhaseSelection(int &errcnt, std::string &errstr)
{
    return cp2130_.getGPIO5(errcnt, errstr);  // GPIO.5 corresponds to the PSEL signal (PSELECT pin on the AD9834 waveform generator)
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

// Selects the active frequency
void GF2Device::selectFrequency(bool select, int &errcnt, std::string &errstr)
{
    cp2130_.setGPIO4(select, errcnt, errstr);  // GPIO.4 corresponds to the FSEL signal (FSELECT pin on the AD9834 waveform generator)
}

// Selects the active phase
void GF2Device::selectPhase(bool select, int &errcnt, std::string &errstr)
{
    cp2130_.setGPIO5(select, errcnt, errstr);  // GPIO.5 corresponds to the PSEL signal (PSELECT pin on the AD9834 waveform generator)
}

// Sets the amplitude of the generated signal to the given value (in Vpp)
void GF2Device::setAmplitude(float amplitude, int &errcnt, std::string &errstr)
{
    if (amplitude < AMPLITUDE_MIN || amplitude > AMPLITUDE_MAX) {
        ++errcnt;
        errstr += "In setAmplitude(): Amplitude must be between 0 and 8.\n";  // Program logic error
    } else {
        cp2130_.selectCS(1, errcnt, errstr);  // Enable the chip select corresponding to channel 1, and disable any others
        uint16_t amplitudeCode = static_cast<uint16_t>(amplitude * AQUANTUM / AMPLITUDE_MAX + 0.5);
        std::vector<uint8_t> setAmplitude = {
            static_cast<uint8_t>(0x0f & amplitudeCode >> 6),  // Amplitude
            static_cast<uint8_t>(amplitudeCode << 2)
        };
        cp2130_.spiWrite(setAmplitude, EPOUT, errcnt, errstr);  // Set the amplitude of the output signal (AD5310 on channel 1)
        usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
        cp2130_.disableCS(1, errcnt, errstr);  // Disable the previously enabled chip select
    }
}

// Sets the frequency, selected by the boolean variable "select", to the given value (in KHz)
void GF2Device::setFrequency(bool select, float frequency, int &errcnt, std::string &errstr)
{
    if (frequency < FREQUENCY_MIN || frequency > FREQUENCY_MAX) {
        ++errcnt;
        errstr += "In setFrequency(): Frequency must be between 0 and 40000.\n";  // Program logic error
    } else {
        cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
        uint32_t frequencyCode = static_cast<uint32_t>(frequency * FQUANTUM / MCLK + 0.5);
        std::vector<uint8_t> setFrequency = {
            static_cast<uint8_t>((select ? FREQ1 : FREQ0) | (0x3f & frequencyCode >> 8)),   // FREQ0 or FREQ1 register set to the given value, according to the boolean variable "select"
            static_cast<uint8_t>(frequencyCode),
            static_cast<uint8_t>((select ? FREQ1 : FREQ0) | (0x3f & frequencyCode >> 22)),
            static_cast<uint8_t>(frequencyCode >> 14)
        };
        cp2130_.spiWrite(setFrequency, EPOUT, errcnt, errstr);  // Set the selected frequency by updating the above registers (AD9834 on channel 0)
        usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
        cp2130_.disableCS(0, errcnt, errstr);  // Disable the previously enabled chip select
    }
}

// Sets the phase, selected by the boolean variable "select", to the given value (in degrees)
void GF2Device::setPhase(bool select, float phase, int &errcnt, std::string &errstr)
{
    cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
    float phaseMod = std::fmod(phase, 360);  // Calculate the remainder of the division between the phase and 360
    uint16_t phaseCode = static_cast<uint16_t>((phaseMod + (phaseMod < 0 ? 360 : 0)) * PQUANTUM / 360 + 0.5);
    std::vector<uint8_t> setPhase = {
        static_cast<uint8_t>((select ? PHASE1 : PHASE0) | (0x0f & phaseCode >> 8)),   // PHASE0 or PHASE1 register set to the given value, according to the boolean variable "select"
        static_cast<uint8_t>(phaseCode)
    };
    cp2130_.spiWrite(setPhase, EPOUT, errcnt, errstr);  // Set the selected phase by updating the above registers (AD9834 on channel 0)
    usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
    cp2130_.disableCS(0, errcnt, errstr);  // Disable the previously enabled chip select
}

// Sets the waveform of the generated signal to sinusoidal
void GF2Device::setSineWave(int &errcnt, std::string &errstr)
{
    cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
    std::vector<uint8_t> setSineWave = {
        0x22, 0x00  // B28 = 1, PIN/SW = 1, MODE = 0 (sinusoidal waveform)
    };
    cp2130_.spiWrite(setSineWave, EPOUT, errcnt, errstr);  // Set the waveform to sinusoidal (AD9834 on channel 0)
    usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
    cp2130_.disableCS(0, errcnt, errstr);  // Disable the previously enabled chip select
}

// Sets the waveform of the generated signal to triangular
void GF2Device::setTriangleWave(int &errcnt, std::string &errstr)
{
    cp2130_.selectCS(0, errcnt, errstr);  // Enable the chip select corresponding to channel 0, and disable any others
    std::vector<uint8_t> setTriangleWave = {
        0x22, 0x02  // B28 = 1, PIN/SW = 1, MODE = 1 (triangular waveform)
    };
    cp2130_.spiWrite(setTriangleWave, EPOUT, errcnt, errstr);  // Set the waveform to triangular (AD9834 on channel 0)
    usleep(100);  // Wait 100us, in order to prevent possible errors while disabling the chip select (workaround)
    cp2130_.disableCS(0, errcnt, errstr);  // Disable the previously enabled chip select
}

// Sets up channel 0 for communication with the AD9834 waveform generator
void GF2Device::setupChannel0(int &errcnt, std::string &errstr)
{
    CP2130::SPIMode mode;
    mode.csmode = CP2130::CSMODEPP;  // Chip select pin mode regarding channel 0 is push-pull
    mode.cfrq = CP2130::CFRQ12M;  // SPI clock frequency set to 12MHz
    mode.cpol = CP2130::CPOL1;  // SPI clock polarity is active low (CPOL = 1)
    mode.cpha = CP2130::CPHA0;  // SPI data is valid on each falling edge (CPHA = 0)
    cp2130_.configureSPIMode(0, mode, errcnt, errstr);  // Configure SPI mode for channel 0, using the above settings
    cp2130_.disableSPIDelays(0, errcnt, errstr);  // Disable all SPI delays for channel 0
}

// Sets up channel 1 for communication with the AD5310 DAC
void GF2Device::setupChannel1(int &errcnt, std::string &errstr)
{
    CP2130::SPIMode mode;
    mode.csmode = CP2130::CSMODEPP;  // Chip select pin mode regarding channel 1 is push-pull
    mode.cfrq = CP2130::CFRQ12M;  // SPI clock frequency set to 12MHz
    mode.cpol = CP2130::CPOL0;  // SPI clock polarity is active high (CPOL = 0)
    mode.cpha = CP2130::CPHA1;  // SPI data is valid on each falling edge (CPHA = 1)
    cp2130_.configureSPIMode(1, mode, errcnt, errstr);  // Configure SPI mode for channel 1, using the above settings
    cp2130_.disableSPIDelays(1, errcnt, errstr);  // Disable all SPI delays for channel 1
}

// Enables or disables the synchronous clock
void GF2Device::switchClock(bool value, int &errcnt, std::string &errstr)
{
    cp2130_.setGPIO6(!value, errcnt, errstr);  // GPIO.6 corresponds to the !CMPEN signal and is connected to the SHDN pin on the TLV3501 comparator
}

// Enables or disables the DAC internal to the AD9834 waveform generator
void GF2Device::switchDAC(bool value, int &errcnt, std::string &errstr)
{
    cp2130_.setGPIO3(!value, errcnt, errstr);  // GPIO.3 corresponds to the SLP signal and is connected to the SLEEP pin on the AD9834 waveform generator
}

// Helper function that returns the expected amplitude from a given amplitude value
// Note that the function is only valid for values between "AMPLITUDE_MIN" [0] and "AMPLITUDE_MAX" [8]
float GF2Device::expectedAmplitude(float amplitude)
{
    return std::round(amplitude * AQUANTUM / AMPLITUDE_MAX) * AMPLITUDE_MAX / AQUANTUM;
}

// Helper function that returns the expected frequency from a given frequency value
// Note that the function is only valid for values between "FREQUENCY_MIN" [0] and "FREQUENCY_MAX" [40000]
float GF2Device::expectedFrequency(float frequency)
{
    return std::round(frequency * FQUANTUM / MCLK) * MCLK / FQUANTUM;
}

// Helper function that returns the expected phase from a given phase value
float GF2Device::expectedPhase(float phase)
{
    float phaseMod = std::fmod(phase, 360);  // Calculate the remainder of the division between the phase and 360
    return std::round((phaseMod + (phaseMod < 0 ? 360 : 0)) * PQUANTUM / 360) * 360 / PQUANTUM;
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
