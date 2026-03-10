#include <ESP32Servo.h>

//----------PIN DECLARATION----------
const int NumServos = 7;
Servo Servos[NumServos];
int PinServos[] = {23, 22, 19, 18, 5, 17, 16};

//----------LIMITS AND HOME ANGLES----------
int MinLim[] = {90, 10, 10, 10, 5, 5, 5}; 
int MaxLim[] = {150, 170, 150, 150, 155, 155, 165};
int Home[] = {150, 90, 150, 80, 150, 50, 85};

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
  //----------SERIAL BEGIN----------
  if (Serial.available() > 0)
  {
    String data = Serial.readStringUntil('\n');
    data.trim(); 

    //----------DATA RECIEVE----------
    if (data.length() > 0)
    {
      int servoIndex = 0;
      int lastIndex = 0;

      //----------PROCESSING----------
      for (int i = 0; i <= data.length() && servoIndex < NumServos; i++)
      {
        if (i == data.length() || data.charAt(i) == ',')
        {
          String part = data.substring(lastIndex, i);
          part.trim();
          
          //----------ANGLE OBTAINANCE----------
          if (part.length() > 0)
          {
            int angle = part.toInt();

            //----------ANGLE WRITE----------
            if (angle > 10 && angle >= MinLim[servoIndex] && angle <= MaxLim[servoIndex])
            {
              Servos[servoIndex].write(angle);
            }
            else
            {
              //----------ANGLE RETAINED----------
              Serial.print("ANGLE RETAINED");
              Serial.print(servoIndex);
              Serial.print("val: ");
              Serial.println(angle);
            }
          }
          lastIndex = i + 1;
          servoIndex++;
        }
      }
    }
  }
}