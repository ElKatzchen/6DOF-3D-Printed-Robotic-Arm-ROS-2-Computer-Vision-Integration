#include <ESP32Servo.h>

//----------PIN DECLARATION----------
const int NumServos = 7;
Servo Servos[NumServos];
int PinServos[] = {32, 33, 25, 19, 18, 5, 27};

//----------LIMITS AND HOME ANGLES----------
int MinLim[] = {90, 10, 10, 10, 5, 5, 5}; 
int MaxLim[] = {150, 170, 170, 150, 175, 175, 165};
int Home[] = {150, 90, 150, 80, 150, 50, 85};

//----------ANGLES GIVEN----------
int g_grip, g_s1, g_s2, g_s3, g_s4, g_s5, g_s6;

//----------SETUP----------
void setup()
{
  //----------SERIAL SETUP----------
  Serial.begin(115200); 
  delay(1000);
  Serial.println("SYSTEM INITIALYZED");

  //----------TIMERS ACTIVATION----------
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  //----------SERVOS ACTIVATION----------
  for (int i = 0; i < NumServos; i++)
  {
    Servos[i].setPeriodHertz(50);
    Servos[i].write(Home[i]);
    Servos[i].attach(PinServos[i], 500, 2400); 
  }
}

//----------LOOP----------
void loop()
{
  if (Serial.available() > 0)
  {
    if (Serial.peek() == '$')
    {
      Serial.read();
      
      String data = Serial.readStringUntil('\n');
      data.trim();
      
      //----------DATA FILTER----------
      if (data.length() < 20) return; 

      int DataCount = 0;
      int startIndex = 0;
      int endIndex = 0;

      //----------DATA LECTURE----------
      while ((endIndex = data.indexOf('/', startIndex)) != -1)
      {
        int FinalValue = data.substring(startIndex, endIndex).toInt();
        
        if (DataCount == 2) g_grip = FinalValue;
        else if (DataCount == 3) g_s1 = FinalValue;
        else if (DataCount == 4) g_s2 = FinalValue;
        else if (DataCount == 5) g_s3 = FinalValue;
        else if (DataCount == 6) g_s4 = FinalValue;
        else if (DataCount == 7) g_s5 = FinalValue;
        else if (DataCount == 8) g_s6 = FinalValue;

        startIndex = endIndex + 1;
        DataCount++;
      }

      if (DataCount == 8)
      {
        g_s6 = data.substring(startIndex).toInt();
        DataCount++;
      }

      //----------SERVO WRITE----------
      if (DataCount >= 9)
      {
          g_grip = constrain(g_grip, MinLim[0], MaxLim[0]);
          g_s1   = constrain(g_s1,   MinLim[1], MaxLim[1]);
          g_s2   = constrain(g_s2,   MinLim[2], MaxLim[2]);
          g_s3   = constrain(g_s3,   MinLim[3], MaxLim[3]);
          g_s4   = constrain(g_s4,   MinLim[4], MaxLim[4]);
          g_s5   = constrain(g_s5,   MinLim[5], MaxLim[5]);
          g_s6   = constrain(g_s6,   MinLim[6], MaxLim[6]);

          Servos[0].write(g_grip);
          Servos[1].write(g_s1);
          Servos[2].write(g_s2);
          Servos[3].write(g_s3);
          Servos[4].write(g_s4);
          Servos[5].write(g_s5);
          Servos[6].write(g_s6);
          
          char answer[64]; 
          sprintf(answer, "ACK:%03d/%03d/%03d/%03d/%03d/%03d/%03d", 
                  g_grip, g_s1, g_s2, g_s3, g_s4, g_s5, g_s6);
          Serial.println(answer);
      }
    } 
    else
    {
      Serial.read();
    }
  }
}