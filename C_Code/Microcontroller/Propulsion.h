#ifndef PROPULSION_H
#define PROPULSION_H

#include <Arduino.h>
#include <Servo.h>

class Propulsion {
public:
    Propulsion(int pin);
    void begin();
    // Cette fonction reçoit une commande en pourcentage de vitesse moteur(-100 à +100)
    void setSpeed(int percent);
    float getSpeed() const;
    void update();

private:
    int motorPin;
    Servo esc;
    int speedPWM; // Valeur PWM actuelle
    static constexpr int neutralPWM = 90;   // PWM neutre (0% de commande)
    static constexpr int maxDeviation = 35; // Écart maximum pour ±100%
    static constexpr float maxSpeed = 50.0;   // Vitesse maximale en km/h
};

#endif
