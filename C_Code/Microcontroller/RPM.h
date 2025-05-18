#ifndef RPM_H
#define RPM_H

#include <Arduino.h>

class RPM {
public:
    RPM(int pin);
    void begin();
    int getRPM();
    void update();

private:
    int sensorPin;
    volatile int pulseCount;
    unsigned long lastTime;
    static constexpr int interval = 1000; // Temps d'échantillonnage en ms
    static constexpr int SEUIL_BAS = 220;
    static constexpr int SEUIL_HAUT = 250;
    bool etatBas;
};

#endif
