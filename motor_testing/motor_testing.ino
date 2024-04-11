#include <DualMAX14870MotorShield.h>
DualMAX14870MotorShield motors;

void setup() {
  // put your setup code here, to run once:
  motors.enableDrivers();
}

void loop() {
  // put your main code here, to run repeatedly:
  motors.setM1Speed(200);
  motors.setM2Speed(200);
  delay(3000);
  motors.setM1Speed(0);
  motors.setM2Speed(0);
  delay(1000);
}
