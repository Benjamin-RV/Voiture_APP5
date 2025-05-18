#include "RPM.h"

RPM::RPM(int pin) : sensorPin(pin), pulseCount(0), lastTime(0), etatBas(false) {}

void RPM::begin() {
    pinMode(sensorPin, INPUT);
    Serial.println("🚀 Initialisation du capteur RPM...");
}

void RPM::update() {
    int sensorValue = analogRead(sensorPin);
    
    if (sensorValue < SEUIL_BAS && !etatBas) {
        pulseCount++;
        etatBas = true;
    }
    if (sensorValue > SEUIL_HAUT) {
        etatBas = false;
    }
}

int RPM::getRPM() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= interval) {
        int rpm = (pulseCount * 60) / 1; // 1 represente le nombre de fentes du disque
        pulseCount = 0;
        lastTime = currentTime;
        return rpm;
    }
    return -1; 
}
