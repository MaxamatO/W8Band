#include "W8BandLauncher.hpp"
#include <Arduino.h>
#include <optional>

std::optional<w8band::W8BandLauncher> launcher;

// Miga N razy, potem dłuższa pauza - żeby dało się policzyć.
// Uwaga: na wielu XIAO LED_BUILTIN jest aktywny w stanie LOW (LOW = zapalona).
static void Checkpoint(int n)
{
    for(int i = 0; i < n; i++)
    {
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
    }
    delay(1000);
}

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    delay(10);

    // CHECKPOINT 1: setup() w ogole wystartowal
    // Checkpoint(1);

    Serial.begin(115200);
    // CHECKPOINT 2: po Serial.begin()
    // Checkpoint(2);

    uint32_t t = millis();
    while(!Serial && (millis() - t < 2000))
    {
        delay(10);
    }
    // CHECKPOINT 3: po petli czekania na Serial
    // Checkpoint(3);

    Serial.println("Halo");
    Serial.flush();
    // CHECKPOINT 4: po probie wypisania Halo
    // Checkpoint(4);
    Wire.begin();
    launcher.emplace();
    // CHECKPOINT 5: po skonstruowaniu launchera
    // Checkpoint(5);

    if(!launcher->Initialize())
    {
        // CHECKPOINT 9: Initialize() zwrocilo false
        while(1)
        {
            Checkpoint(9);
        }
    }
    // CHECKPOINT 6: Initialize() sie udalo
    // Checkpoint(6);

    launcher->LaunchW8Band();
    // CHECKPOINT 7: po LaunchW8Band()
    // Checkpoint(7);
}

void loop()
{
    // CHECKPOINT 8: w loop() - jesli tu dotarles, wszystko wystartowalo
    // Serial.println("update");
    // Serial.flush();
    launcher->Update();
    // delay(10);
}