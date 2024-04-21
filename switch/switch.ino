#define switch_pin 2
#define goal_led 50
int goalState;

void setup()
{
  Serial.begin(9600);
  pinMode(switch_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(switch_pin), read_switch_state, CHANGE);
  goalState = digitalRead(switch_pin);  
}

void loop(){
  Serial.println(goalState);
}

void read_switch_state() {
  goalState = digitalRead(switch_pin);
  if (goalState == 0)
  {
    digitalWrite(goal_led, LOW);
  } 
  else
  {
    digitalWrite(goal_led, HIGH);
  }   
}

