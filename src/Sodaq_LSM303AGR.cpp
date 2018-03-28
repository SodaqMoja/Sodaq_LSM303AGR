#include <Arduino.h>
#include <math.h>
#include "Sodaq_LSM303AGR.h"

#define _BV(bit) (1 << (bit))

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Sodaq_LSM303AGR::Sodaq_LSM303AGR(TwoWire& wire, uint8_t accelAddress, uint8_t magAddress) :
    _wire(wire),
    _accelAddress(accelAddress),
    _magAddress(magAddress),
    _accelScale(Scale2g)
{

}

int8_t Sodaq_LSM303AGR::getTemperatureDelta()
{
    setAccelRegisterBits(CTRL_REG4_A, _BV(BDU));

    readAccelRegister(OUT_TEMP_L_A); // low byte can be thrown away, but must be read!
    int8_t value = readAccelRegister(OUT_TEMP_H_A);

    unsetAccelRegisterBits(CTRL_REG4_A, _BV(BDU));

    return value;
}

void Sodaq_LSM303AGR::setAccelScale(Scale scale)
{
    writeAccelRegister(CTRL_REG4_A, (scale << FS0));
    _accelScale = scale;
}

double Sodaq_LSM303AGR::getGsFromScaledValue(int16_t value)
{
    int8_t scaleMax = getAccelScaleMax(_accelScale);
    return mapDouble(value, INT16_MIN, INT16_MAX, -scaleMax, scaleMax);
}

double Sodaq_LSM303AGR::getMagFromScaledValue(int16_t value)
{
    return value * 1.5;
}

int16_t Sodaq_LSM303AGR::getScaledValueFromGs(double gValue)
{
    int8_t scaleMax = getAccelScaleMax(_accelScale);
    return trunc(mapDouble(gValue, -scaleMax, scaleMax, INT16_MIN, INT16_MAX));
}

int8_t Sodaq_LSM303AGR::getAccelScaleMax(Scale scale)
{
    return (1 << (scale + 1));
}

bool Sodaq_LSM303AGR::checkWhoAmI()
{
    return (readAccelRegister(WHO_AM_I_A) == 0b00110011) &&
            (readMagRegister(WHO_AM_I_M) == 0b01000000);
}

void Sodaq_LSM303AGR::enableAccelerometer(AccelerometerMode mode, AccelerometerODR odr, Axes axes, Scale scale, bool isTemperatureOn)
{
    // set odr, mode, enabled axes
    // Note: the values of AccelerometerMode are 0b(LPen,HR)
    writeAccelRegister(CTRL_REG1_A, (odr << ODR0) | ((mode & 0b10) << LPen) | axes);
    if (mode & 0b01) {
        setAccelRegisterBits(CTRL_REG4_A, _BV(HR));
    }
    else {
        unsetAccelRegisterBits(CTRL_REG4_A, _BV(HR));
    }

    setAccelScale(scale);

    if (isTemperatureOn) {
        // enable aux ADC and temperature sensor
        writeAccelRegister(TEMP_CFG_REG_A, _BV(TEMP_EN1) | _BV(TEMP_EN0));
    }
    else {
        // disable aux ADC and temperature sensor
        writeAccelRegister(TEMP_CFG_REG_A, 0);
    }
}

void Sodaq_LSM303AGR::enableMagnetometer(MagnetometerMode mode, MagnetometerODR odr, MagnetometerSystemMode systemMode, bool compensateTemp, bool enableLPF)
{
    // set odr, mode, systemMode
    writeMagRegister(CFG_REG_A_M, (odr << MagODR0) | systemMode);

    if (mode == MagLowPowerMode) {
        setMagRegisterBits(CFG_REG_A_M, _BV(LP));
    }
    else {
        unsetMagRegisterBits(CFG_REG_A_M, _BV(LP));
    }

    if (compensateTemp) {
        setMagRegisterBits(CFG_REG_A_M, _BV(COMP_TEMP_EN));
    }
    else {
        unsetMagRegisterBits(CFG_REG_A_M, _BV(COMP_TEMP_EN));
    }

    // disable hard-iron calibration
    writeMagRegister(OFFSET_X_REG_L_M, 0);
    writeMagRegister(OFFSET_X_REG_H_M, 0);
    writeMagRegister(OFFSET_Y_REG_L_M, 0);
    writeMagRegister(OFFSET_Y_REG_H_M, 0);
    writeMagRegister(OFFSET_Z_REG_L_M, 0);
    writeMagRegister(OFFSET_Z_REG_H_M, 0);

    // disable offset cancellation
    unsetMagRegisterBits(CFG_REG_B_M, _BV(OFF_CANC));

    setLPF(enableLPF);
}

void Sodaq_LSM303AGR::setLPF(bool enabled)
{
    if (enabled) {
        setMagRegisterBits(CFG_REG_B_M, _BV(LPF));
    }
    else {
        unsetMagRegisterBits(CFG_REG_B_M, _BV(LPF));
    }
}

void Sodaq_LSM303AGR::disableAccelerometer()
{
    enableAccelerometer(LowPowerMode, PowerDown, NoAxis, _accelScale, false);
}

void Sodaq_LSM303AGR::disableMagnetometer()
{
    enableMagnetometer(MagLowPowerMode, Hz10, IdleMode, false);
}

void Sodaq_LSM303AGR::rebootAccelerometer()
{
    writeAccelRegister(CTRL_REG5_A, _BV(BOOT));
}

void Sodaq_LSM303AGR::rebootMagnetometer()
{
    writeMagRegister(CFG_REG_A_M, _BV(REBOOT));
}

void Sodaq_LSM303AGR::setRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue)
{
    uint8_t value = readRegister(deviceAddress, reg);
    value |= byteValue;
    writeRegister(deviceAddress, reg, value);
}

void Sodaq_LSM303AGR::unsetRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue)
{
    uint8_t value = readRegister(deviceAddress, reg);
    value &= ~byteValue;
    writeRegister(deviceAddress, reg, value);
}

void Sodaq_LSM303AGR::enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    // setup the interrupt
    writeAccelRegister(INT1_CFG_A, interruptMode | (axesEvents & 0b00111111));
    writeAccelRegister(INT1_THS_A, trunc(mapDouble(threshold, 0, getAccelScaleMax(_accelScale), 0, INT8_MAX)));
    writeAccelRegister(INT1_DURATION_A, duration); // time duration is INT1_DURATION_A/ODR

    // disable latching
    unsetAccelRegisterBits(CTRL_REG5_A, _BV(LIR_IG1));

    // enable interrupt generator 1 on INT1
    setAccelRegisterBits(CTRL_REG3_A, _BV(INT1_AOI1));
}

void Sodaq_LSM303AGR::disableInterrupt1()
{
    // disable interrupt generator 1
    unsetAccelRegisterBits(CTRL_REG3_A, _BV(INT1_AOI1));
}

void Sodaq_LSM303AGR::enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    // setup the interrupt
    writeAccelRegister(INT2_CFG_A, interruptMode | (axesEvents & 0b00111111));
    writeAccelRegister(INT2_THS_A, trunc(mapDouble(threshold, 0, getAccelScaleMax(_accelScale), 0, INT8_MAX)));
    writeAccelRegister(INT2_DURATION_A, duration);  // time duration is INT2_DURATION_A/ODR

    // disable latching
    unsetAccelRegisterBits(CTRL_REG5_A, _BV(LIR_IG2));

    // enable interrupt generator 2 on INT2
    setAccelRegisterBits(CTRL_REG6_A, _BV(I2_INT2));
}

void Sodaq_LSM303AGR::disableInterrupt2()
{
    // disable interrupt generator 2 on INT2
    unsetAccelRegisterBits(CTRL_REG6_A, _BV(I2_INT2));
}


void Sodaq_LSM303AGR::enableMagnetometerInterrupt(uint8_t magAxesEvents, double threshold, bool highOnInterrupt)
{
    // threshold needs to be positive, because mag checks interrupts for -threshold and +threshold always
    if (threshold < 0) {
        threshold = -threshold;
    }

    // set axes
    writeMagRegister(INT_CTRL_REG_M, (magAxesEvents << ZIEN));

    // interrupt mode
    if (highOnInterrupt) {
        setMagRegisterBits(INT_CTRL_REG_M, _BV(IEA));
    }
    else {
        unsetMagRegisterBits(INT_CTRL_REG_M, _BV(IEA));
    }
    
    // set threshold registers
    int16_t ths = trunc(threshold / 1.5);
    writeMagRegister(INT_THS_L_REG_M, ths & 0x00FF);
    writeMagRegister(INT_THS_H_REG_M, (ths & 0xFF00) >> 8);

    // disable latching
    unsetMagRegisterBits(INT_CTRL_REG_M, _BV(IEL));

    // enable mag interrupt
    setMagRegisterBits(INT_CTRL_REG_M, _BV(IEN));

    // set mag interrupt to INT_MAG_PIN
    setMagRegisterBits(CFG_REG_C_M, _BV(INT_MAG_PIN));
}

void Sodaq_LSM303AGR::disableMagnetometerInterrupt()
{
    // disable mag interrupt
    unsetMagRegisterBits(INT_CTRL_REG_M, _BV(IEN));
}

uint8_t Sodaq_LSM303AGR::readRegister(uint8_t deviceAddress, uint8_t reg)
{
    _wire.beginTransmission(deviceAddress);
    _wire.write((uint8_t)reg);
    _wire.endTransmission();

    _wire.requestFrom(deviceAddress, 1);

    return _wire.read();
}

uint16_t Sodaq_LSM303AGR::readRegister16Bits(uint8_t deviceAddress, uint8_t reg)
{
    // TODO replace with request of 2 bytes?
    // TODO: don't we need BDU Here?
    uint16_t result = readRegister(deviceAddress, reg);
    result |= readRegister(deviceAddress, reg + 1) << 8;

    return result;
}

void Sodaq_LSM303AGR::writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(deviceAddress);

    _wire.write((uint8_t)reg);
    _wire.write((uint8_t)value);

    _wire.endTransmission();
}
