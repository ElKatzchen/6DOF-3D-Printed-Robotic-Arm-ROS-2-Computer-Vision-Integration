#include <ESP32Servo.h>

const int NumServos = 7;
Servo Servos[NumServos];
int PinServos[] = {32, 33, 25, 19, 18, 5, 27};

int MinLim[] = {30, 10, 10, 10, 5, 5, 5}; 
int MaxLim[] = {90, 170, 170, 150, 175, 175, 165};
int Home[] = {90, 90, 150, 80, 150, 50, 85};

void setup()
{
  Serial.begin(115200); 
  delay(1000);
  Serial.println("SYSTEM INITIALYZED");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NumServos; i++)
  {
    Servos[i].setPeriodHertz(50);
    Servos[i].write(Home[i]);
    Servos[i].attach(PinServos[i], 500, 2400); 
  }
}

void loop()
{
  if (Serial.available() > 0)
  {
    if (Serial.peek() == '$')
    {
      Serial.read();
      
      String data = Serial.readStringUntil('\n');
      data.trim();
      
      if (data.length() < 15) return; 

      int DataCount = 0;
      int startIndex = 0;
      int endIndex = 0;
      int val;

      while ((endIndex = data.indexOf('/', startIndex)) != -1)
      {
        if (DataCount < 7) {
            val = data.substring(startIndex, endIndex).toInt();
            Servos[DataCount].write(constrain(val, MinLim[DataCount], MaxLim[DataCount]));
            DataCount++;
        }
        startIndex = endIndex + 1;
      }

      if (DataCount < 7)
      {
        val = data.substring(startIndex).toInt();
        Servos[DataCount].write(constrain(val, MinLim[DataCount], MaxLim[DataCount]));
        DataCount++;
      }
      
      if (DataCount >= 7) {
          Serial.println("ACK:OK");
      }
    } 
    else
    {
      Serial.read();
    }
  }
}