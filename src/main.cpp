#include "w8band.hpp"
#include <Arduino.h>
#include <LSM6DSV16XSensor.h>
#include <Wire.h>

w8band::W8Band Band(Wire);

void setup()
{
    Serial.begin(115200);
    while(!Serial)
    {
        yield();
    }
    Wire.begin();
    Band.Init();
}

void loop()
{
    Band.Update();
    delay(100);
}