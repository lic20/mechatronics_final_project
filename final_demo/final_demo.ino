#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <Pixy2.h>
#include <math.h>
#include <DualMAX14870MotorShield.h>
#include <Servo.h>

//potentiometers
#define pot1 A7
#define pot2 A8
#define pot3 A9

//Ultrasonic Ping sensor
#define sonar 13
//IR sensors right(r) and left(l) from back
#define ir_r A2
#define ir_l A15

#define servo_r 22
#define servo_l 23

int puck_area = 0;
int puck_angle = 0;

int pink_goal[2] = {210, 60};
int red_goal[2] = {30, 60};
int robot_loc[2] = {0, 0};

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

double heading = 0.0;
int forward_speed = 120;
int direction = 0;
double tolerance = 8.0;
int turning = 0;

Pixy2 pixy;
DualMAX14870MotorShield motors;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Servo servo1;
Servo servo2;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(115200);
  Serial.begin(115200);
  pixy.init();

  /* Initialise the imu sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  servo1.attach(servo_r);
  servo2.attach(servo_l);
  motors.enableDrivers();
  bno.setExtCrystalUse(true);
  delay(1000);
}

void loop() {
  // int index = getBlock();
  // read_servo(servo_r);
  // read_servo(servo_l);
  // measureDistance();

  // move_servo();
  // getBlock();

  track_puck();
  delay(3000);
  // while(true) {
  //   stop();
  // }
}


bool find_puck() {
  double current_heading = read_imu();
  int M1 = 80;
  int M2 = -80;
  unsigned long start = millis();
  motors.setM1Speed(M1);
  motors.setM2Speed(M2);

  bool found = false;
  while(true) {
    int index = getBlock();

    if (index >= 0) {
      found = true;
      break;
    }
    if (start - millis() > 5000) {
      break;
    }
  }
  stop();
  return found;
}

void track_puck(){
  find_puck();
  delay(10);
  heading = heading + turning;
  anglerange();
  delay(10);
  PID_move(heading, 1);
  stop();
}

int getBlock()
{
  int markers_num = pixy.ccc.getBlocks(); //get detected blocks from Pixy
  Serial.println("Number of blocks: " + String(markers_num));

  // if no maker is detected, return -1
  if (markers_num == 0) {
    Serial.println("No markers detected");
    return -1;
  }

  int x_center = 158;
  int y_center = 104;
  // if (markers_num == 1) {
  //   int x = pixy.ccc.blocks[0].m_x;
  //   puck_angle = int((float)(x_center-x)/5.2667);
  //   puck_area = pixy.ccc.blocks[0].m_width * pixy.ccc.blocks[0].m_height;
  //   return 0;
  // }

  int max_area = 0;
  int max_type = -1;
  //loops through all detected blocks:
  for (int i=0;i < markers_num; i++)
  { 
    int x = pixy.ccc.blocks[i].m_x; int y = pixy.ccc.blocks[i].m_y;
    Serial.println("i =" + String(i) + ", x: " + String(x) + ", y: " + String(y));
    int y_offset = y_center-y;
    int x_offset = x_center-x;
    Serial.println("x_offset: " + String(x_offset) + ", y_offset: " + String(y_offset));
    // exclude the blocks that are far off from the center horizontal line
    if (y_offset < -50){
      continue;
    }
    int area = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height;
    
    // get the largest area
    if (area > max_area){
      max_area = area;
      max_type = i;
      //negative: turn left, positive: turn right
      turning = int((float)x_offset/5.2667);
      Serial.println(String(turning));
    }
  }

  puck_area = pixy.ccc.blocks[max_type].m_width * pixy.ccc.blocks[max_type].m_height;
  puck_angle = turning;
  int color = pixy.ccc.blocks[max_type].m_signature;
  Serial.println("Detected signature: " + String(color) + ", " + String(max_type));
  return max_type;
}

int get_data() {
  // send data to XBee
  // char req = '?';
  Serial1.println("?");
  //get data from XBee
  Serial.println("requested data, now getting data...");
  String input;
  input.reserve(200);
  int count = 0;
  while (Serial1.available()) {
    char in = Serial1.read();
    if (String(in) == ",") {
      if (count == 0) {
        Serial.println(input);
        if (String(input) == "0" || String(input) == "?") {
          Serial.println("No match or data is invalid!!");
          return -1;
        }
      }
      else if (count == 1) {
        Serial.println("Time: " + input);
      }
      else if (count == 2) {
        if (String(input) == "---") {
          Serial.println("Data is invalid!!");
          return -1;
        }
        Serial.println("X loc: " + String(input));
        robot_loc[0] = input.toInt();
      }
      count ++;
      input = "";
    }
    else {  
      input += in;
    }
  }
  if (input != "") {
    Serial.println("Y loc: " + String(input));
    robot_loc[1] = input.toInt();
  }
  return 0;
}

void move_servo() {
  Serial.println("Moving servo.....");
  int init_pos = 10;
  int final_pos = 90;
  servo1.write(60);
  servo2.write(115);
  delay(2000);
  servo1.write(140);
  servo2.write(30);
  Serial.println("Move complete.....");
  delay(2000);
}

void read_servo(int pin) {
  Serial.println("Servo 1 pos: " + String(servo1.read()));
  Serial.println("Servo 2 pos: " + String(servo2.read()));
}

double read_imu()
{
  sensors_event_t event;
  bno.getEvent(&event);
  
  return event.orientation.x;
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

void move_forward(int offset, int turning)
{ 
  if (turning == 0) { //move left
    int M1 = speedrange(forward_speed - offset, 0);
    int M2 = speedrange(forward_speed + offset, 0);
    Serial.println("Straight => Move left: M1=" + String(M1) + ", M2=" + String(M2) );
    motors.setM1Speed(M1);
    motors.setM2Speed(M2);
  }
  else if (turning == 1) { //move right
    int M1 = speedrange(forward_speed + offset, 0);
    int M2 = speedrange(forward_speed -offset, 0);
    Serial.println("Straight => Move right: M1=" + String(M1) + ", M2=" + String(M2) );
    motors.setM1Speed(M1);
    motors.setM2Speed(M2);
  }
}

void stop(){
  motors.setM1Speed(0);
  motors.setM2Speed(0);
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
      move_forward(offset,direction);
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
    // bool first_turn = true;
    while (error > tolerance){
      int offset = PID_control(desired_angle, current_angle, error);
      if (offset < 0) {
        current_angle = read_imu();
        error = shortest_angle(current_angle, desired_angle);
        continue;
      }
      Serial.println("Desired angle: " + String(desired_angle) + ", Current angle: " + String(current_angle));
      Serial.println("PID offset: " + String(offset));
      // if (first_turn) {
      //   if (direction == 0){
      //     move_left(offset);
      //   }
      //   else if (direction == 1) {
      //     move_right(offset);
      //   }
      //   first_turn = false;
      // }
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
      current_angle = read_imu();
      error = shortest_angle(current_angle, desired_angle);
    }
    stop();
    delay(1000);
  }
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
 Serial.println("Distance: "+String(distance));
 return distance;
}

void reset_PID() {
  integral = 0;
  derivative = 0;
  previous_error = 0;
}

double shortest_angle(double an1, double an2)
{
  double clockwise = abs(an1-an2);
  double counterclockwise = 360.0 - clockwise;
  return min(clockwise, counterclockwise);

}

int speedrange(int n, int mode){
  int val = n;
  int max_val = 80;
  if (mode == 0) {
    max_val = 200;
  }
  if (n > max_val){
    val = max_val;
  }
  else if (n < -max_val){
    val = -max_val;
  }
  return val;
}

void anglerange() {
  if (heading >  360.0) {
    heading = heading - 360.0;
  }
  else if (heading < 0.0){
    heading = 360.0 + heading;
  }
}

void setPID(int motion) {
  if (motion == 0) //forward straight
  {
    Kp = 4.54; Ki = 0.0; Kd = 0.12;
    //read potentiometer values
    // int pot1_val = analogRead(pot1);
    // int pot2_val = analogRead(pot2);
    // int pot3_val = analogRead(pot3);
    // Kp = ((double)pot1_val*7.0)/1023.0;
    // Kd = ((double)pot3_val*3.0)/1023.0;
    // Ki = ((double)pot2_val*0.25)/1023.0;
    // Serial.println("Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd:" + String(Kd));
  }
  else if (motion == 1) // turning
  {
    Kp = 6.85; Ki = 0.0; Kd = 0.68;
    // int pot1_val = analogRead(pot1);
    // int pot2_val = analogRead(pot2);
    // int pot3_val = analogRead(pot3);
    // Kp = ((double)pot1_val*7.0)/1023.0;
    // Kd = ((double)pot3_val*3.0)/1023.0;
    // Ki = ((double)pot2_val*0.25)/1023.0;
    Serial.println("Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd:" + String(Kd));
  }
}