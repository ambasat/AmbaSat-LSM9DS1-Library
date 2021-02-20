/*******************************************************************************
* AmbaSat-1 LSM9DS1 9DOF sensor. Accelerometer, Gyroscope and Magnetometer

* 20th February 2021
* Version 1.0
* Filename: main.cpp
*
* Copyright (c) 2020 AmbaSat Ltd
* https://ambasat.com
*
* Licensed under Creative Commons Attribution-ShareAlike 3.0

* ******************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <AmbaSatLSM9DS1.h>
#include <Debugging.h>

AmbaSatLSM9DS1 *ambasatLSM9DS1;

#define LSM9DS1_M	0x1E  // Mag address
#define LSM9DS1_AG	0x6B  // AG address

// ==============================================================================
void setup() 
{
    Serial.begin(9600);
    while (!Serial);

    ambasatLSM9DS1 = new AmbaSatLSM9DS1();

    if (ambasatLSM9DS1->begin(LSM9DS1_AG, LSM9DS1_M) == false) 
    {
        PRINTLN_DEBUG(F("Failed to communicate with the AmbaSat-1 LSM9DS1 IMU"));
        while (1);
    }
    else
    {
        PRINTLN_DEBUG(F("AmbaSat-1 LSM9DS1 IMU communication established"));
    }
}

// ==============================================================================
void loop()
{
    ambasatLSM9DS1->readGyro();
    ambasatLSM9DS1->readAccel();
    ambasatLSM9DS1->readMag();

    PRINT_DEBUG(F("GYRO: "));
    PRINT_DEBUG(ambasatLSM9DS1->gx);
    PRINT_DEBUG(F(", "));
    PRINT_DEBUG(ambasatLSM9DS1->gy);
    PRINT_DEBUG(F(", "));
    PRINTLN_DEBUG(ambasatLSM9DS1->gz);    

    PRINT_DEBUG(F("ACCEL: "));
    PRINT_DEBUG(ambasatLSM9DS1->ax);
    PRINT_DEBUG(F(", "));
    PRINT_DEBUG(ambasatLSM9DS1->ay);
    PRINT_DEBUG(F(", "));
    PRINTLN_DEBUG(ambasatLSM9DS1->az);      

    PRINT_DEBUG(F("MAG: "));
    PRINT_DEBUG(ambasatLSM9DS1->mx);
    PRINT_DEBUG(F(", "));
    PRINT_DEBUG(ambasatLSM9DS1->my);
    PRINT_DEBUG(F(", "));
    PRINTLN_DEBUG(ambasatLSM9DS1->mz);  

    delay(1000);
}
