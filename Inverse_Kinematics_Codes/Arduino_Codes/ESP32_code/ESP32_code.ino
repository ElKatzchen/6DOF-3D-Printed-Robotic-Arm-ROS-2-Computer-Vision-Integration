#include <ESP32Servo.h>

//GRIPPER 90---150 open-close
//JOINT 1 90+-80 ccw-cw
//JOINT 2 80+-70 up down
//JOINT 3 80+-70 ccw-cw
//JOINT 4 80+-75 up-down
//JOINT 5 80+-75 up-down
//JOINT 6 85+-80 ccw-cw

const int NumServos = 7;
Servo Servos[NumServos];
int PinServos[] = {23, 22, 19, 18, 5, 17, 16};

int MinLim[] = {90, 10, 10, 10, 5, 5, 5}; 
int MaxLim[] = {150, 170, 150, 150, 155, 155, 165};
int Home[] = {150, 90, 150, 80, 150, 50, 85};

void setup() {
  Serial.begin(115200); 
  delay(1000);
  Serial.println("SYSTEM INITIALYZED");

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NumServos; i++) {
    Servos[i].setPeriodHertz(50);
    Servos[i].attach(PinServos[i], 500, 2400); 
    Servos[i].write(Home[i]);
  }
}

void loop() {
  if (Serial.available() > 0) {
    if (isDigit(Serial.peek()) || Serial.peek() == '-') {
      
      for (int i = 0; i < NumServos; i++) {
        int Angle = Serial.parseInt();
        
        int SafeSpot = constrain(Angle, MinLim[i], MaxLim[i]);
        Servos[i].write(SafeSpot);

        while (Serial.available() > 0 && (Serial.peek() == ',' || Serial.peek() == ' ')) {
          Serial.read();
        }
      }

      while (Serial.available() > 0) {
        Serial.read();
      }

    } else {
      Serial.read();
    }
  }

  Serial.println("OK");
}