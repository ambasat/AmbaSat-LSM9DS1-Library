/*******************************************************************************
* AmbaSat-1 
* Filename: AmbaSatLSM9DS1.h
*
* This library is designed specifically for AmbaSat-1 and its onboard
* LSM9DS1 IMU
* 
* Copyright (c) 2021 AmbaSat Ltd
* https://ambasat.com
*
* licensed under Creative Commons Attribution-ShareAlike 3.0
* ******************************************************************************/

#ifndef __AmbaSatLSM9DS1__
#define __AmbaSatLSM9DS1__

#include "Arduino.h"
#include <Wire.h>
#include <Debugging.h>

// ag
#define LSM9DS1_WHO_AM_I           0x0f
#define LSM9DS1_CTRL_REG1_G        0x10
#define LSM9DS1_CTRL_REG3_G        0x12
#define LSM9DS1_OUT_X_G            0x18
#define LSM9DS1_CTRL_REG6_XL       0x20
#define LSM9DS1_OUT_X_XL           0x28

// m
#define LSM9DS1_CTRL_REG2_M        0x21
#define LSM9DS1_CTRL_REG8          0x22
#define LSM9DS1_CTRL_REG3_M        0x22
#define LSM9DS1_CTRL_REG4_M        0x23
#define LSM9DS1_OUT_X_L_M          0x28

class AmbaSatLSM9DS1 
{
    public:
    	int16_t gx, gy, gz; // gyroscope x, y, and z axis readings 
	    int16_t ax, ay, az; // accelerometer x, y, and z axis readings  
	    int16_t mx, my, mz; // magnetometer x, y, and z axis readings 

        AmbaSatLSM9DS1(void);   
    	bool begin(uint8_t agAddress, uint8_t mAddress);         
        bool readGyro(void);    
        bool readAccel(void);
        bool readMag(void);
    private:
        bool i2cInitialised = false;
        uint8_t _agAddress;	 
	    uint8_t _mAddress;	 
        virtual uint8_t i2cAutoIncrementBit(void) const { return 0; }        
        bool writeRegister(uint8_t deviceAddress, uint8_t address, uint8_t value);
        bool readRegisterValue(uint8_t deviceAddress, uint8_t address, uint8_t& register_value);  
        bool readRegisterData(uint8_t deviceAddress, uint8_t address, uint8_t* data, size_t length);  
};

#endif // __AmbaSatLSM9DS1__