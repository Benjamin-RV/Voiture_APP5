#ifndef DIRECTION_H
#define DIRECTION_H

#include <Arduino.h>
#include <Servo.h>

class Direction {
public:
    Direction(int pin);
    void begin();
    void setAnglePWM(int pwmValue);
    float getAngle() const;
    void update();

private:
    int steerPin;
    Servo cmd_direction;
    int anglePWM;

    // Définition des limites PWM pour la direction
    static constexpr int centerPWM = 90;  // Position neutre du servo (0°)
    static constexpr int minPWM = 60;     // Braquage à gauche (-30°)
    static constexpr int maxPWM = 120;    // Braquage à droite (+30°)
    static constexpr int maxAngle = 30;   // Amplitude max en degrés
};

#endif
