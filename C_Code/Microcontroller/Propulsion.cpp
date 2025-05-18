#include "Propulsion.h"

Propulsion::Propulsion(int pin) : motorPin(pin), speedPWM(neutralPWM) {}

void Propulsion::begin() {
    esc.attach(motorPin);
    Serial.println("Initialisation de l'ESC...");
    esc.write(neutralPWM);  // Position neutre pour éviter un démarrage brusque
    delay(3000);
}

void Propulsion::setSpeed(int percent) {
    Serial.print("Commande de vitesse reçue (en pourcentage) : ");
    Serial.println(percent);

    // Conversion du pourcentage en valeur PWM :
    // 0% correspond à neutralPWM, +100% à neutralPWM + maxDeviation, -100% à neutralPWM - maxDeviation
    int pwmValue = neutralPWM + (percent * maxDeviation) / 100;
    
    // Contrainte de la valeur PWM entre neutralPWM - maxDeviation et neutralPWM + maxDeviation
    int minPWM = neutralPWM - maxDeviation;  // 55
    int maxPWM = neutralPWM + maxDeviation;  // 125
    pwmValue = constrain(pwmValue, minPWM, maxPWM);

    // Gestion du frein pour la transition vers la marche arrière
    if (pwmValue < neutralPWM && speedPWM >= neutralPWM) {
        Serial.println("🚨 Activation du frein avant la marche arrière...");
        esc.write(neutralPWM);
        delay(1000);  // Pause pour permettre la transition
    }

    speedPWM = pwmValue;
    update();
}

float Propulsion::getSpeed() const {
    // Conversion de la commande PWM en vitesse estimée en km/h
    if (speedPWM > neutralPWM) {
        return (speedPWM - neutralPWM) * (maxSpeed / static_cast<float>(maxDeviation)); // Marche avant
    } else if (speedPWM < neutralPWM) {
        return -(neutralPWM - speedPWM) * (maxSpeed / static_cast<float>(maxDeviation)); // Marche arrière (vitesse négative)
    }
    return 0.0; // Arrêt
}

void Propulsion::update() {
    // Mise à jour de l'ESC si la valeur change
    if (esc.read() != speedPWM) {
        esc.write(speedPWM);
    }
    Serial.print("Commande envoyée à l'ESC : PWM ");
    Serial.print(speedPWM);
    Serial.print(" | Vitesse réelle estimée : ");
    Serial.print(getSpeed());
    Serial.println(" km/h");
}
