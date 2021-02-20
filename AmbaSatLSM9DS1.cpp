/*******************************************************************************
* AmbaSat-1 
* Filename: AmbaSatLSM9DS1.cpp
*
* This library is designed specifically for AmbaSat-1 and its onboard
* LSM9DS1 IMU
* 
* Copyright (c) 2021 AmbaSat Ltd
* https://ambasat.com
*
* licensed under Creative Commons Attribution-ShareAlike 3.0
* ******************************************************************************/

#include "AmbaSatLSM9DS1.h"

// ======================================================================================

AmbaSatLSM9DS1::AmbaSatLSM9DS1(void) 
{
    if (!i2cInitialised) 
    {
        PRINTLN_DEBUG(F("Starting I2C interface."));
        Wire.begin(); 
        delay(250); // Wait for sensor to start
        
        // Global I2C reset
        PRINTLN_DEBUG(F("Global I2C reset"));
        Wire.beginTransmission(0x00); // global i2c reset address
        Wire.write(0x06);
        Wire.endTransmission(); 
        
        delay(50); // wait for everything to start
        PRINTLN_DEBUG(F("I2C Wire has been set up."));
        i2cInitialised = true;
    }
}

// ======================================================================================

bool AmbaSatLSM9DS1::begin(uint8_t agAddress, uint8_t mAddress) 
{
	_agAddress = agAddress;
	_mAddress = mAddress;
    uint8_t register_value;

    if (!writeRegister(_agAddress, LSM9DS1_CTRL_REG8, 0x05)) 
    {
        return false;
    }

    writeRegister(_mAddress, LSM9DS1_CTRL_REG2_M, 0x0c);

    delay(10);

    if (!readRegisterValue(_agAddress, LSM9DS1_WHO_AM_I, register_value) || (register_value != 0x68)) 
    {
        return false;
    }

    if (!readRegisterValue(_mAddress, LSM9DS1_WHO_AM_I, register_value) || (register_value != 0x3d)) 
    {
        return false;
    }

    // Gyro:
    //      * 59.5 Hz ODR (CTRL_REG1_G | 0b01000000)
    //      * 245 DPS scale (CTRL_REG1_G | 0b00000000)
    //      * Low Power (CTRL_REG3_G bit LP_mode set to 1 ==> 0b10000000 )
    writeRegister(_agAddress, LSM9DS1_CTRL_REG1_G, 0x40);
    writeRegister(_agAddress, LSM9DS1_CTRL_REG3_G, 0x80);
  
    // Accel:
    //      * 50 Hz ODR (CTRL_REG6_XL | 0b01000000)
    //      * 2G scale (CTRL_REG6_XL | 0b00000000)
    writeRegister(_agAddress, LSM9DS1_CTRL_REG6_XL, 0x40);
    
    //  Magneto:
    //      * Low power mode (CTRL_REG3_M | 0b00100000) and (CTRL_REG4_M | 0b00000000)
    //      * Continuous conversion (CTRL_REG3_M | 0b00000000)
    //      * 4 gauss (CTRL_REG2_M | 0b00000000)
    //      * Z-axis operative mode selection low power (CTRL_REG4_M | 0b00000000)
    //      * Big Endian (CTRL_REG4_M | 0b00000000)
    writeRegister(_mAddress, LSM9DS1_CTRL_REG3_M, 0x20); 
    writeRegister(_mAddress, LSM9DS1_CTRL_REG2_M, 0x00); 
    writeRegister(_mAddress, LSM9DS1_CTRL_REG4_M, 0x00);

    // TODO: sensitivity could be set here

    return true;    
}

// ======================================================================================

bool AmbaSatLSM9DS1::writeRegister(uint8_t deviceAddress, uint8_t address, uint8_t value)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    Wire.write(value);

    if (Wire.endTransmission() != 0) 
    {
        PRINTLN_DEBUG(F("Error: endTransmission with address: "));
        PRINTLN_DEBUG(deviceAddress);
        return false;
    }

    return true;
}

// ======================================================================================

bool AmbaSatLSM9DS1::readRegisterValue(uint8_t deviceAddress, uint8_t address, uint8_t& register_value)
{

    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    
    if (Wire.endTransmission() != 0) 
    {
        PRINTLN_DEBUG(F("Error: endTransmission with address: "));
        PRINTLN_DEBUG(deviceAddress);
       return false;
    }

    if (Wire.requestFrom(deviceAddress, (uint8_t)1) != 1) 
    {
        PRINTLN_DEBUG(F("ERROR: requestFrom with address: "));
        PRINTLN_DEBUG(deviceAddress);
        return false;
    }

    int value = Wire.read();

    if (Wire.endTransmission() != 0) 
    {
        PRINTLN_DEBUG(F("ERROR: endTransmission with address: "));
        PRINTLN_DEBUG(deviceAddress);
        return false;
    }

    register_value = value;
    return true;
}

// ======================================================================================

bool AmbaSatLSM9DS1::readRegisterData(uint8_t deviceAddress, uint8_t address, uint8_t* data, size_t length)
{
    Wire.beginTransmission(deviceAddress);

    uint8_t autoIncrementBit = 0x00;

    if (i2cAutoIncrementBit() > 0 ) 
    {
        autoIncrementBit = 1 << i2cAutoIncrementBit();
    }

    Wire.write(autoIncrementBit | address);

    if (Wire.endTransmission(false) != 0) 
    {
        PRINTLN_DEBUG(F("ERROR: endTransmission with address: "));
        PRINTLN_DEBUG(deviceAddress);
        return false;
    }

    if (Wire.requestFrom(deviceAddress, length) != length) 
    {
        PRINTLN_DEBUG(F("ERROR: requestFrom with address: "));
        PRINTLN_DEBUG(deviceAddress);
        return false;
    }

    for (size_t i = 0; i < length; i++) 
    {
        *data++ = Wire.read();
    }

    if (Wire.endTransmission() != 0) 
    {
        PRINTLN_DEBUG(F("ERROR: endTransmission with address: "));
        PRINTLN_DEBUG(deviceAddress);
        return false;
    }

    return true;
}

// ======================================================================================

bool AmbaSatLSM9DS1::readGyro(void)
{
	uint8_t gyroData[6]; //  six gyro data bytes

    if (!readRegisterData(_agAddress, LSM9DS1_OUT_X_G, (uint8_t*)gyroData, sizeof(gyroData))) 
    {
        PRINTLN_DEBUG(F("ERROR reading LSM9DS1 gyro data"));
        return false;        
    }
    else
    {
		gx = (gyroData[1] << 8) | gyroData[0]; 
		gy = (gyroData[3] << 8) | gyroData[2]; 
		gz = (gyroData[5] << 8) | gyroData[4]; 

        return true;
    }
}

// ======================================================================================

bool AmbaSatLSM9DS1::readAccel(void)
{
	uint8_t accelData[6]; //  six accel data bytes

    if (!readRegisterData(_agAddress, LSM9DS1_OUT_X_XL, (uint8_t*)accelData, sizeof(accelData))) 
    {
        PRINTLN_DEBUG(F("ERROR reading LSM9DS1 accel data"));
        return false;        
    }
    else
    {
		ax = (accelData[1] << 8) | accelData[0]; 
		ay = (accelData[3] << 8) | accelData[2]; 
		az = (accelData[5] << 8) | accelData[4]; 

        return true;
    }
}

// ======================================================================================

bool AmbaSatLSM9DS1::readMag(void)
{
	uint8_t magData[6]; //  six mag data bytes

    if (!readRegisterData(_mAddress, LSM9DS1_OUT_X_L_M, (uint8_t*)magData, sizeof(magData))) 
    {
        PRINTLN_DEBUG(F("ERROR reading LSM9DS1 magData data"));
        return false;        
    }
    else
    {
		mx = (magData[1] << 8) | magData[0]; 
		my = (magData[3] << 8) | magData[2]; 
		mz = (magData[5] << 8) | magData[4]; 

        return true;
    }
}
