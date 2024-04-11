//Ping sensor
//Connect Ping signal pin to Arduino digital 8
//Mechatronics 530.421 Lab1
int signal=13;
int distance;
unsigned long pulseduration=0;
void setup()
{
 pinMode(signal, OUTPUT);
 Serial.begin(9600);
}
void measureDistance()
{
 // set pin as output so we can send a pulse
 pinMode(signal, OUTPUT);
// set output to LOW
 digitalWrite(signal, LOW);
 delayMicroseconds(5);
 
 // now send the 5uS pulse out to activate Ping)))
 digitalWrite(signal, HIGH);
 delayMicroseconds(5);
 digitalWrite(signal, LOW);
 
 // now we need to change the digital pin
 // to input to read the incoming pulse
 pinMode(signal, INPUT);
 
 // finally, measure the length of the incoming pulse
 pulseduration=pulseIn(signal, HIGH);
}
void loop()
{
 // get the raw measurement data from Ping)))
 measureDistance();
 //************************//
 // FILL IN YOUR CODES HERE
 distance = pulseduration /29.15 / 2;
 // END OF YOUR CODES
 //************************//
 // Display on serial monitor
 Serial.print("Distance - ");
 Serial.print(distance);
 Serial.println(" cm");
 delay(50);
}
