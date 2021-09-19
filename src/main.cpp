#include <Arduino.h>
#include <heater1.h>
#include "GyverPID.h"

class ImitHeatObj {
  public:
    float value = 0;

    ImitHeatObj(float ENV_T, int DELAY_AMOUNT, float SIGNAL_COEF, float COEF) 
    {
      this->ENV_T = ENV_T; // Температура окружающей среды
      this->value = ENV_T; 
      this->DELAY_AMOUNT = DELAY_AMOUNT; // Задержка изменений (сколько шагов не будет видно изменений при воздействии на обьект)
      this->delayArray = new float[DELAY_AMOUNT];
      this->SIGNAL_COEF = SIGNAL_COEF; // Сила сигнала
      this->COEF = COEF; // Множитель температуры
    }

    void calc(float signal) 
    {
      // signal == скорость нагрева, ограничивает сигнал его же значением и плавно к нему стремится
      signalSpeed += (signal - signalSpeed) * 0.003;

      // Складываем скорость сигнала и скорость охлаждения
      // Скорость охлаждения получаем как разность "температуры" и её нулевого значения
      valueSpeed = signalSpeed * SIGNAL_COEF + (ENV_T - value) * COEF;

      if (!firstFlag) {
        firstFlag = true;
        for (int i = 0; i < DELAY_AMOUNT; i++) {
          delayArray[i] = valueSpeed;
        }
      }

      for (int i = 0; i < DELAY_AMOUNT - 1; i++){
        delayArray[i] = delayArray[i + 1];
      } 

      delayArray[DELAY_AMOUNT - 1] = valueSpeed;

      // Прибавляем скорость (интегрируем)
      value += delayArray[0]; //valueSpeed
    }

    String getSerialData() {
      return "imitHeatObj1/setTemperature/" + String(value);
    }

  private:
    int DELAY_AMOUNT; 
    float SIGNAL_COEF; 
    float COEF; 
    float ENV_T; 
    float valueSpeed;
    float signalSpeed;
    bool firstFlag = false;
    float *delayArray;
};


ImitHeatObj *imitHeatObj1;

// Heater1* heater1;

GyverPID regulator1(4.2, 0.6, 0);

void setup() {
  imitHeatObj1 = new ImitHeatObj(24.0, 10, 0.1, 0.1);
  Serial.begin(9600);
  regulator1.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator1.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator1.setpoint = 160;
}

String strData = "";
boolean recievedFlag;


String getTypePayloadWrapper1(String &type, String &payload) 
{
  String result = type + '|' + payload;
  return result;
}

#define DT 100

String type1 = String("heater1/setCurrentTemperature");

bool regulator1_active = false;

void regulator1_setParams(float point, float Kp, float Ki, float Kd) {
  regulator1.setpoint = point;
  regulator1.Kp = Kp;
  regulator1.Ki = Ki;
  regulator1.Kd = Kd;
}

void loop() {
  static uint32_t tmr;
  static uint32_t tmr1;

  if (millis() - tmr >= DT) {
    tmr = millis();

    regulator1.input = imitHeatObj1->value;
    
    if (regulator1.integral < 0) {
      regulator1.integral = 0;
    }

    if (regulator1_active) {
      imitHeatObj1->calc(regulator1.getResult());
    } else {
      imitHeatObj1->calc(0);
    }


  }

  // To SERIAL
  if (millis() - tmr1 >= 1000) {
    tmr1 = millis();
    Serial.println("PIDRegulator1/setSignal/" + String(regulator1.output));
    Serial.println(imitHeatObj1->getSerialData());
  }

  while (Serial.available() > 0) {           
    strData += (char)Serial.read();       
    recievedFlag = true;                 
    delay(2);                           
  }

  if (recievedFlag) {
    if (strData.startsWith("|") && strData.endsWith("|")) {
      strData.replace("|", "");

      int addressDelimiter = strData.indexOf("/");
      int actionDelimiter = strData.indexOf("/", addressDelimiter + 1);
      
      String address = strData.substring(0, addressDelimiter);
      String action = strData.substring(addressDelimiter + 1, actionDelimiter);
      String values = strData.substring(actionDelimiter + 1);

      if (address == "PIDRegulator1") {
        if (action == "setParams") {

          int activeDelimiter = values.indexOf(";");
          int pointDelimiter = values.indexOf(";", activeDelimiter + 1);
          int KpDelimiter = values.indexOf(";", pointDelimiter + 1);
          int KiDelimiter = values.indexOf(";", KpDelimiter + 1);
          
          bool active = (bool)values.substring(0, activeDelimiter).toInt();
          float point = values.substring(activeDelimiter + 1, pointDelimiter).toFloat();
          float Kp = values.substring(pointDelimiter + 1, KpDelimiter).toFloat();
          float Ki = values.substring(KpDelimiter + 1, KiDelimiter).toFloat();
          float Kd = values.substring(KiDelimiter + 1).toFloat();
          
          regulator1_active = active;
          regulator1_setParams(point, Kp, Ki, Kd);

          Serial.println("PIDRegulator1/setParams/OK");
        }
      }
    }
    strData = ""; 
    recievedFlag = false;                  
  }
}




