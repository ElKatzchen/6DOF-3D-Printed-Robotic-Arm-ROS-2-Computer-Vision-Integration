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

// GPIO 23, 22, 19, 18, 5, 17 (TX2), 16 (RX2)
int PinServos[] = {23, 22, 19, 18, 5, 17, 16};

//-----SERVOS SETTING-----
//Change this to the default setting for the servos
int Set[] = {90, 90, 150, 80, 150, 50, 85}; 

void setup() {
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  for (int i = 0; i < NumServos; i++) {
    Servos[i].setPeriodHertz(50);
    Servos[i].attach(PinServos[i], 500, 2400); 
    Servos[i].write(Set[i]);
  }
}

void loop() {
  delay(1000); 
}