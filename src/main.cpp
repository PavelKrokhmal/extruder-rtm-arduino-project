#include <Arduino.h>
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
      this->value += delayArray[0]; //valueSpeed
    }

  private:
    int DELAY_AMOUNT; 
    float SIGNAL_COEF; 
    float COEF; 
    float ENV_T; 
    float valueSpeed;
    float signalSpeed = 0;
    bool firstFlag = false;
    float *delayArray;
};


ImitHeatObj *imitHeatObj1;
ImitHeatObj *imitHeatObj2;

GyverPID regulator1(4.2, 0.6, 0);
GyverPID regulator2(4.2, 0.6, 0);

void setup() {
  Serial.begin(9600);

  imitHeatObj1 = new ImitHeatObj(24.0, 10, 0.1, 0.1);
  imitHeatObj2 = new ImitHeatObj(24.0, 5, 0.1, 0.1);
  
  regulator1.setDirection(NORMAL);
  regulator1.setLimits(0, 255);
  regulator1.setpoint = 150;

  regulator2.setDirection(NORMAL);
  regulator2.setLimits(0, 255);
  regulator2.setpoint = 150;
}

String strData = "";
boolean recievedFlag;


String getTypePayloadWrapper1(String &type, String &payload) 
{
  String result = type + '|' + payload;
  return result;
}

bool regulator1_active = false;
bool regulator2_active = false;

void regulator_setParams(GyverPID* regulator, float point, float Kp, float Ki, float Kd) {
  regulator->setpoint = point;
  regulator->Kp = Kp;
  regulator->Ki = Ki;
  regulator->Kd = Kd;
}

void loop() {
  static uint32_t tmr;
  static uint32_t tmr1;

  if (millis() - tmr >= 100) {
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

    regulator2.input = imitHeatObj2->value;
    
    if (regulator2.integral < 0) {
      regulator2.integral = 0;
    }

    if (regulator2_active) {
      imitHeatObj2->calc(regulator2.getResult());
    } else {
      imitHeatObj2->calc(0);
    }
  }

  // To SERIAL PORT
  if (millis() - tmr1 >= 1000) {
    tmr1 = millis();
    Serial.println("PIDRegulator1/setSignal/" + String(regulator1.output));
    Serial.println("imitHeatObj1/setTemperature/" + String(imitHeatObj1->value));
    Serial.println("PIDRegulator2/setSignal/" + String(regulator2.output));
    Serial.println("imitHeatObj2/setTemperature/" + String(imitHeatObj2->value));
    Serial.println("sensors/setTemperature1/" + String(imitHeatObj2->value - 5));
    Serial.println("sensors/setLevel1/" + String(60));
    Serial.println("sensors/setThickness1/" + String(2.75));
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

      if (address == "PIDRegulator1" && action == "setParams") {
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
          regulator_setParams(&regulator1, point, Kp, Ki, Kd);

          Serial.println("PIDRegulator1/setParams/OK");
      }

       if (address == "PIDRegulator2" && action == "setParams") {

          int activeDelimiter = values.indexOf(";");
          int pointDelimiter = values.indexOf(";", activeDelimiter + 1);
          int KpDelimiter = values.indexOf(";", pointDelimiter + 1);
          int KiDelimiter = values.indexOf(";", KpDelimiter + 1);
          
          bool active = (bool)values.substring(0, activeDelimiter).toInt();
          float point = values.substring(activeDelimiter + 1, pointDelimiter).toFloat();
          float Kp = values.substring(pointDelimiter + 1, KpDelimiter).toFloat();
          float Ki = values.substring(KpDelimiter + 1, KiDelimiter).toFloat();
          float Kd = values.substring(KiDelimiter + 1).toFloat();
          
          regulator2_active = active;
          regulator_setParams(&regulator2, point, Kp, Ki, Kd);

          Serial.println("PIDRegulator2/setParams/OK");
      }
    }
    strData = ""; 
    recievedFlag = false;                  
  }
}