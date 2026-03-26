#include <ESP32Servo.h>

//----------PIN DECLARATION----------
const int NumServos = 7;
Servo Servos[NumServos];
int PinServos[] = {23, 22, 19, 18, 5, 17, 16};

//----------LIMITS AND HOME ANGLES----------
int MinLim[] = {90, 10, 10, 10, 5, 5, 5}; 
int MaxLim[] = {150, 170, 150, 150, 155, 155, 165};
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
    //----------BUFFER FISRT CHARACTER DETECTION----------
    if (Serial.peek() == '$')
    {
      Serial.read(); 
      
      String data = Serial.readStringUntil('\n');
      data.trim();
      int n = data.length();
      
      String TemporalBlock = ""; 
      int DataCount = 0;

      //----------MANUAL READ----------
      for (int i = 0; i <= n; i++)
      {
        char c = (i < n) ? data.charAt(i) : '/'; 

        if (c != '/')
        {
          TemporalBlock += c; 
        } 
        else
        {
          int FinalValue = TemporalBlock.toInt();
          
          if (DataCount == 2) g_grip = FinalValue;
          else if (DataCount == 3) g_s1 = FinalValue;
          else if (DataCount == 4) g_s2 = FinalValue;
          else if (DataCount == 5) g_s3 = FinalValue;
          else if (DataCount == 6) g_s4 = FinalValue;
          else if (DataCount == 7) g_s5 = FinalValue;
          else if (DataCount == 8) g_s6 = FinalValue;

          TemporalBlock = "";
          DataCount++;
        }
      }

      //----------SERVO WRITE----------
      if (DataCount >= 9)
      {
          if (g_grip > 10 && g_grip >= MinLim[0] && g_grip <= MaxLim[0]) Servos[0].write(g_grip);
          if (g_s1   > 10 && g_s1   >= MinLim[1] && g_s1   <= MaxLim[1]) Servos[1].write(g_s1);
          if (g_s2   > 10 && g_s2   >= MinLim[2] && g_s2   <= MaxLim[2]) Servos[2].write(g_s2);
          if (g_s3   > 10 && g_s3   >= MinLim[3] && g_s3   <= MaxLim[3]) Servos[3].write(g_s3);
          if (g_s4   > 10 && g_s4   >= MinLim[4] && g_s4   <= MaxLim[4]) Servos[4].write(g_s4);
          if (g_s5   > 10 && g_s5   >= MinLim[5] && g_s5   <= MaxLim[5]) Servos[5].write(g_s5);
          if (g_s6   > 10 && g_s6   >= MinLim[6] && g_s6   <= MaxLim[6]) Servos[6].write(g_s6);
          
          Serial.println("DISPATCH_COMPLETE");
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