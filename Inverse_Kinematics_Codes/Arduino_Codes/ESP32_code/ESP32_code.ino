#include <ESP32Servo.h>

const int NumServos = 7;
Servo Servos[NumServos];
int PinServos[] = {32, 33, 25, 19, 18, 5, 27};

int MinLim[] = {30, 10, 10, 10, 5, 5, 5}; 
int MaxLim[] = {90, 170, 170, 150, 175, 175, 165};
int Home[] = {90, 90, 150, 80, 150, 50, 85};

int g_grip_actual, g_grip_target; 
int g_s1, g_s2, g_s3, g_s4, g_s5, g_s6;

const int Samples = 5;
int Buffer[7][Samples];
int Indice = 0;

unsigned long lastGripTime = 0;
const long gripInterval = 200;

int PromCalculus(int servoIdx, int nuevoValor) {
  Buffer[servoIdx][Indice] = nuevoValor;
  long suma = 0;
  for (int i = 0; i < Samples; i++) {
    suma += Buffer[servoIdx][i];
  }
  return suma / Samples;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NumServos; i++) {
    Servos[i].setPeriodHertz(50);
    Servos[i].write(Home[i]);
    Servos[i].attach(PinServos[i], 500, 2400);
    for(int j = 0; j < Samples; j++) Buffer[i][j] = Home[i];
  }
  g_grip_actual = Home[0];
  g_grip_target = Home[0];
}

void loop() {
  if (millis() - lastGripTime >= gripInterval) {
    if (g_grip_actual != g_grip_target) {
      if (g_grip_actual < g_grip_target) g_grip_actual += 5;
      else g_grip_actual -= 5;
      
      g_grip_actual = constrain(g_grip_actual, MinLim[0], MaxLim[0]);
      Servos[0].write(g_grip_actual);
    }
    lastGripTime = millis();
  }

  if (Serial.available() > 0) {
    if (Serial.peek() == '$') {
      Serial.read();
      String data = Serial.readStringUntil('\n');
      data.trim();
      
      if (data.length() < 15) return;

      int values[7];
      int DataCount = 0;
      int startIndex = 0;
      int endIndex;

      while ((endIndex = data.indexOf('/', startIndex)) != -1) {
        if (DataCount < 7) values[DataCount++] = data.substring(startIndex, endIndex).toInt();
        startIndex = endIndex + 1;
      }
      if (DataCount < 7) values[DataCount++] = data.substring(startIndex).toInt();

      if (DataCount >= 7) {
        // Gripper: sin filtro, va directo al target para la rampa
        g_grip_target = constrain(values[0], MinLim[0], MaxLim[0]);
        
        // Servos 1-6: aplican el filtro de media móvil
        g_s1 = PromCalculus(1, constrain(values[1], MinLim[1], MaxLim[1]));
        g_s2 = PromCalculus(2, constrain(values[2], MinLim[2], MaxLim[2]));
        g_s3 = PromCalculus(3, constrain(values[3], MinLim[3], MaxLim[3]));
        g_s4 = PromCalculus(4, constrain(values[4], MinLim[4], MaxLim[4]));
        g_s5 = PromCalculus(5, constrain(values[5], MinLim[5], MaxLim[5]));
        g_s6 = PromCalculus(6, constrain(values[6], MinLim[6], MaxLim[6]));

        Servos[1].write(g_s1);
        Servos[2].write(g_s2);
        Servos[3].write(g_s3);
        Servos[4].write(g_s4);
        Servos[5].write(g_s5);
        Servos[6].write(g_s6);
        
        Indice = (Indice + 1) % Samples;
      }
    } else {
      Serial.read();
    }
  }
}