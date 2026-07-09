#include "w8band.hpp"
#include <Arduino.h>
#include <LSM6DSV16XSensor.h>
#include <Wire.h>

w8band::W8Band *Band = nullptr;

void setup()
{
    Serial.begin(115200);
    uint32_t t = millis();
    while(!Serial && (millis() - t < 2000))
    {
        yield();
    }
    Wire.begin();
    Band = new w8band::W8Band(Wire);
    Band->Init();
}

void loop() { Band->Update(); }