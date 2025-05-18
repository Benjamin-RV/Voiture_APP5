#include "Ultrason.h"

SRF10::SRF10(uint8_t address) : i2cAddress(address) {}

void SRF10::begin() {
    Wire.begin();
    Serial.println("🚀 Initialisation du capteur SRF10...");
}

int SRF10::getDistance() {
    // 🔹 Demande de mesure en cm
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x00);  // Registre de commande
    Wire.write(0x51);  // Commande pour mesurer en cm
    Wire.endTransmission();

    // 🔹 Attente des données
    delay(70);  

    // 🔹 Vérification que les données sont prêtes
    Wire.beginTransmission(i2cAddress);
    Wire.write(0x02);  // Registre contenant la distance
    Wire.endTransmission();
    Wire.requestFrom(i2cAddress, 2);

    if (Wire.available() >= 2) {
        uint8_t highByte = Wire.read();
        uint8_t lowByte = Wire.read();
        return (highByte << 8) + lowByte;  // Distance en cm
    }
    
    return -1;  
}

bool SRF10::isObstacleDetected(int threshold) {
    int distance = getDistance();
    return (distance > 0 && distance < threshold);
}
