#define switch_pin 2
#define led_pin 52
int goalState;

void setup()
{
  Serial.begin(9600);
  pinMode(switch_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(switch_pin), read_switch_state, CHANGE);
  goalState = digitalRead(switch_pin);
  pinMode(led_pin, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);  
}

void loop(){
  Serial.println(goalState);
 if (goalState == 0)
  {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(led_pin, LOW);
  } 
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(led_pin, HIGH);
  }  
  delay(100);
}

void read_switch_state() {
  goalState = digitalRead(switch_pin); 
}

