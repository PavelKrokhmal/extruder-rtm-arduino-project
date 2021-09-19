#include <heater1.h>

Heater1 :: Heater1()
{
    currentTemperature = 32.0;
}

String getTypePayloadWrapper(String &type, String &payload) 
{
    String result = "{";
    result.concat("\n\"type\": \"");
    result.concat(type);
    result.concat("\",");
    result.concat("\n\"payload\": ");
    result.concat(payload);
    result.concat("\n}");
  return result;
}

String Heater1 :: getSerialData() 
{
    currentTemperature++;

    String type = String("heater1/setCurrentTemperature");
    String action = String(currentTemperature, 2);
    
    return getTypePayloadWrapper(type, action);
}

void Heater1 :: setSettedTemperature(uint8_t value) 
{
    String type = String("heater1/setCurrentTemperature");
    String action = String("OK");
   Serial.println(getTypePayloadWrapper(type, action));
}