
#include "Direction.h"

Direction::Direction(int pin) : steerPin(pin), anglePWM(centerPWM) {}

void Direction::begin() {
    cmd_direction.attach(steerPin);
    Serial.println("Initialisation de la direction...");
    cmd_direction.write(centerPWM);
    delay(500);
}

void Direction::setAnglePWM(int pwmValue) {
    //Serial.print("Commande PWM de direction reçue : ");
    //Serial.println(pwmValue);

    // Assurer que la valeur PWM est entre minPWM et maxPWM
    anglePWM = constrain(pwmValue, minPWM, maxPWM);

    //Serial.print("Nouvel angle PWM appliqué : ");
    //Serial.println(anglePWM);
}

float Direction::getAngle() const {
    // Convertir le PWM en angle réel (-30° à +30°)
    return (anglePWM - centerPWM) * (maxAngle / (float)(maxPWM - centerPWM));
}

void Direction::update() {
    cmd_direction.write(anglePWM);

    //Serial.print("Commande envoyée au servo : PWM ");
    //Serial.print(anglePWM);
    //Serial.print(" | Angle réel estimé : ");
    //Serial.print(getAngle());
    //Serial.println("°");
}
