#ifndef heater1_h
#define heater1_h

#include <Arduino.h>

class Heater1 
{ 
    public:
        Heater1();
        String getSerialData();
        void setSettedTemperature(uint8_t value);
        
    private:
        float currentTemperature;
};

#endif