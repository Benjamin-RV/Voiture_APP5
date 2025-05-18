#include <SoftwareSerial.h>
#include "Propulsion.h"
#include "Direction.h"
#include "Ultrason.h"

const long baudRate = 38400;
SoftwareSerial BTSerial(0, 1);  // RX sur 2, TX sur 3 (HC-05/HC-06)

Propulsion motor(9);  // ESC connecté à la broche 9
Direction steer(10);   // Servo connecté à la broche 10
SRF10 sonar(0x70);     // Capteur SRF10 à l'adresse 0x70

String commandBuffer = "";
bool stop = false;  // État d'arrêt de la voiture
int lastSpeed = 90; // Mémorise la dernière vitesse envoyée

const int targetDistance = 20; // Distance cible du mur en cm
const int tolerance = 1; // Tolérance réduite pour plus de précision
float currentAngle = 90; // Angle actuel de la direction

void setup() {
    Serial.begin(38400);
    BTSerial.begin(baudRate);

    Serial.println("🚀 Démarrage du système...");
    BTSerial.println("🚀 Démarrage du système...");

    motor.begin();
    steer.begin();
    sonar.begin();
}

void loop() {
    // 🔹 Vérifie la distance avec le capteur
    int distance = sonar.getDistance();
    Serial.print("📝 Distance mesurée : ");
    Serial.print(distance);
    Serial.println(" cm");

    // 🔹 Obtenir la vitesse actuelle
    float speed = motor.getSpeed();
    Serial.print("⚡ Vitesse actuelle : ");
    Serial.println(speed);

    // 🔹 Ajustement progressif de la direction en fonction de l'écart
    float error = targetDistance - distance;
    float responsiveness = max(0.2, 1.0 - (abs(speed) / 50.0)); // Moins réactif à haute vitesse
    float correction = -error * responsiveness * 2.5; // Inverser la correction
    
    // Appliquer une correction progressive
    float targetAngle = 90 + correction;
    targetAngle = constrain(targetAngle, 67, 118
    );

    // Ajustement progressif
    currentAngle = currentAngle + 0.4 * (targetAngle - currentAngle);
    steer.setAnglePWM(static_cast<int>(currentAngle));
    steer.update();

    while (BTSerial.available()) { 
        char c = BTSerial.read();
        Serial.print("📱 Caractère reçu : ");
        Serial.println(c);

        if (c == '\n') {  // Fin de la commande
            Serial.print("✅ Commande complète reçue : ");
            Serial.println(commandBuffer);

            int commaIndex = commandBuffer.indexOf(',');
            if (commaIndex != -1) {
                String speedStr = commandBuffer.substring(0, commaIndex);
                String angleStr = commandBuffer.substring(commaIndex + 1);

                int speedPercentage = speedStr.toInt();
                int angleValue = angleStr.toInt();

                if (speedPercentage >= -100 && speedPercentage <= 100 && angleValue >= 60 && angleValue <= 120) {
                    Serial.println("✅ Commande valide, mise à jour des moteurs et de la direction !");
                    motor.setSpeed(speedPercentage);
                    motor.update();
                    lastSpeed = speedPercentage; // Sauvegarde la dernière vitesse en %

                    steer.setAnglePWM(angleValue);
                    steer.update();
                } else {
                    Serial.println("⚠ ERREUR : Commande invalide ! Valeurs hors plage.");
                }
            } else {
                Serial.println("⚠ ERREUR : Format attendu -> Pourcentage,Angle");
            }
            commandBuffer = "";
        } else {
            commandBuffer += c;
        }
    }
}
