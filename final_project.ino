#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <Pixy2.h>
#include <math.h>
#include <DualMAX14870MotorShield.h>

//electronics
//potentiometers
#define pot1 A7
#define pot2 A8
#define pot3 A9
//IR sensors right(r) and left(l) from back
#define ir_r A2
#define ir_l A15
//Ultrasonic Ping sensor
#define sonar 13
//Servo right(r) and left(l) from back
#define servo_r 22
#define servo_l 23
//Button for switching sides
#define button 53

// PID constants
// ---------------
double Kp = 0.0; // Proportional gain
double Ki = 0.0; // Integral gain
double Kd = 0.0; // Derivative gain

// Variables for PID control
double integral = 0;
double derivative;
double previous_error = 0;

// Refresh rate
unsigned long refreshRate = 50; // Refresh rate in milliseconds
unsigned long lastTime = 0;

int pot1_val = 0;
int pot2_val = 0;
int pot3_val = 0;

int forward_speed = 90;
int direction = 0;
double tolerance = 8.0;
double heading = 0.0;

Pixy2 pixy;
DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(55);


void setup() {
  // put your setup code here, to run once:
    // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Starting...\n");
  pinMode(sonar, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  // pinMode(led, OUTPUT);
  pixy.init();

  /* Initialise the imu sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  motors.enableDrivers();
  bno.setExtCrystalUse(true);
  reset_PID();
  pixy.init();
  delay(3000);
}

void loop() {
  // put your main code here, to run repeatedly:

}

//Reset PID values
void reset_PID() {
  integral = 0;
  derivative = 0;
  previous_error = 0;
}

int PID_control(double desired_angle, double current_angle, double error){
  unsigned long now = millis();
  if (now - lastTime >= refreshRate) {
    lastTime = now;
    int output = 0;
    // Calculate PID
    Serial.println("error: "+ String(error));
    integral += error * (refreshRate / 1000.0);
    derivative = (error - previous_error) / (refreshRate / 1000.0);
    output = Kp * error + Ki * integral + Kd * derivative;
    previous_error = error;
    return abs(output);
  }
  return -1;
}

//Calculate IR voltage from IR Right
void ir_right_distance() {
  float volts_r = analogRead(ir_r)*0.0048828125;  // value from sensor * (5/1024)
  //Serial.println(volt_r);
  delay(200); // small delay
}

//Calculate IR voltage from IR Left
void ir_left_distance() {
  float volts_l = analogRead(ir_l)*0.0048828125;  // value from sensor * (5/1024)
  //Serial.println(volts); 
  delay(200); // small delay
}

int speedrange(int n, int mode){
  int val = n;
  int max_val = 80;
  if (mode == 0) {
    max_val = 120;
  }
  if (n > max_val){
    val = max_val;
  }
  else if (n < -max_val){
    val = -max_val;
  }
  return val;
}

//Keep values between 0 and 360 degrees
void anglerange() {
  if (heading >  360.0) {
    heading = heading - 360.0;
  }
  else if (heading < 0.0){
    heading = 360.0 + heading;
  }
}

double shortest_angle(double an1, double an2)
{
  double clockwise = abs(an1-an2);
  double counterclockwise = 360.0 - clockwise;
  return min(clockwise, counterclockwise);

}

double read_imu()
{
  sensors_event_t event;
  bno.getEvent(&event);
  
  return event.orientation.x;
}

//Measure distance using the ultrasonic sensor
double measureDistance()
{
 // set pin as output so we can send a pulse
 pinMode(sonar, OUTPUT);
// set output to LOW
 digitalWrite(sonar, LOW);
 delayMicroseconds(2);
 
 // now send the 5uS pulse out to activate Ping)))
 digitalWrite(sonar, HIGH);
 delayMicroseconds(5);
 digitalWrite(sonar, LOW);
 
 // now we need to change the digital pin
 // to input to read the incoming pulse
 pinMode(sonar, INPUT);
 
 // finally, measure the length of the incoming pulse and convert it to cm
 unsigned long pulseduration =pulseIn(sonar, HIGH);
 double distance = pulseduration /29.15 / 2;
 // Serial.println("Distance: "+String(distance));
 return distance;
}

//Update what the Pixy sees
int getBlock()
{
  int markers_num = pixy.ccc.getBlocks(); //get detected blocks from Pixy
  Serial.println("Number of blocks: " + String(markers_num));

  // if no maker is detected, return -1
  if (markers_num == 0) {
    Serial.println("No markers detected");
    return -1;
  }
  if (markers_num == 1) {
    return 0;
  }

  int max_area = 0;
  int x_center = 158;
  int y_center = 104;
  int max_type = 0;
  //loops through all detected blocks:
  for (int i=0;i < markers_num; i++)
  { 
    int x = pixy.ccc.blocks[i].m_x; int y = pixy.ccc.blocks[i].m_y;
    int y_offset = y_center-y;
    int x_offset = x_center-x;
    // exclude the blocks that are far off from the center horizontal line
    if (abs(y_offset) > 30 || abs(x_offset) > 50){
      continue;
    }
    int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[0].m_height;
    
    // get the largest area
    if (area > max_area){
      max_area = area;
      max_type = i;
    }
  }

  int color = pixy.ccc.blocks[max_type].m_signature;
  // Serial.println("Detected signature: " + String(color));
  return max_type;
}

//Stop the robot
void stop(){
  motors.setM1Speed(0);
  motors.setM2Speed(0);
}

void move_left(int offset)
{
  int M1 = speedrange(-offset,1);
  int M2 = speedrange(offset,1);
  motors.setM1Speed(M1);
  motors.setM2Speed(M2);
}

void move_right(int offset)
{
  int M1 = speedrange(offset,1);
  int M2 = speedrange(-offset,1);
  motors.setM1Speed(M1);
  motors.setM2Speed(M2);
}

void approach()
{
  double distance = 0.0;
  distance = measureDistance();
  Serial.println("Approaching marker....");
  // moving forward until the robot is less than or equal to 10 cm away from a wall
  while (1)
  {
    PID_move(heading,0);
    distance = measureDistance();
    if (distance <= 8.0){
      stop();
      delay(800);
      distance = measureDistance();
      if (distance <= 8.0) {
        break;
      }
    }
    delay(50);
  }
  stop();
  delay(100);
}

void PID_move(double desired_angle, int mode){
  //PID control and apply the output of PID
  if (mode == 0) {
    Serial.println("PID: moving straight");
    setPID(0);
    delay(5);
    double current_angle = read_imu();
    double error = shortest_angle(current_angle, desired_angle);
    int offset = PID_control(desired_angle, current_angle,error);
    Serial.println("Offset = " + String(offset));
    if (offset >= 0) {
      Serial.println("Desired angle: " + String(desired_angle) + ", Current angle: " + String(current_angle));
      Serial.println("PID offset: " + String(offset));
      int direction = 0;
      if (abs(desired_angle-360.0) < 0.01 || abs(desired_angle-0.0) < 0.01){
        if (current_angle <= 180.0) {
          direction = 0;
        }
        else {
          direction = 1;
        }
      }
      else{
        if (current_angle - desired_angle > 0.0) {
          direction = 0;
        }
        else {
          direction = 1;
        }
      }
      move_forward(offset,direction); //Error on this line. New Move_forward function. No measure for move back.
    }
  }
  else if (mode == 1){
    Serial.println("PID: turning");
    reset_PID();
    delay(5);
    setPID(1);
    delay(5);
    double current_angle = read_imu();
    double error = shortest_angle(current_angle, desired_angle);
    bool first_turn = true;
    while (error > tolerance){
      int offset = PID_control(desired_angle, current_angle, error);
      if (offset < 0) {
        current_angle = read_imu();
        error = shortest_angle(current_angle, desired_angle);
        continue;
      }
      Serial.println("Desired angle: " + String(desired_angle) + ", Current angle: " + String(current_angle));
      Serial.println("PID offset: " + String(offset));
      if (first_turn) {
        if (direction == 0){
          move_left(offset);
        }
        else if (direction == 1) {
          move_right(offset);
        }
        first_turn = false;
      }
      else {
        int direction = 0;
        if (abs(desired_angle-360.0) < 0.01 || abs(desired_angle-0.0) < 0.01){
          if (current_angle <= 180.0) {
            direction = 0;
          }
          else {
            direction = 1;
          }
        }
        else{
          if (current_angle - desired_angle > 0.0) {
            direction = 0;
          }
          else {
            direction = 1;
          }
        }
        if (direction == 0) { //move left
          move_left(offset);
        }
        else if (direction == 1){
          move_right(offset); //move right
        }
      }
      current_angle = read_imu();
      error = shortest_angle(current_angle, desired_angle);
    }
    stop();
    delay(1000);
  }
}

void setPID(int motion) {
  if (motion == 0) //forward straight
  {
    //read potentiometer values
    pot1_val = analogRead(pot1);
    pot2_val = analogRead(pot2);
    pot3_val = analogRead(pot3);
    Kp = ((double)pot1_val*7.0)/1023.0;
    Kd = ((double)pot3_val*3.0)/1023.0;
    Ki = ((double)pot2_val*0.25)/1023.0;
  }
  else if (motion == 1) // turning
  {
    Kp = 6.13; Ki = 0.03; Kd = 0.53;
  }
}
