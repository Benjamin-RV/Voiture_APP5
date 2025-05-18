#ifndef SRF10_H
#define SRF10_H

#include <Arduino.h>
#include <Wire.h>

class SRF10 {
public:
    SRF10(uint8_t address = 0x70);
    
    void begin();
    int getDistance();
    bool isObstacleDetected(int threshold = 10);

private:
    uint8_t i2cAddress;
};

#endif
