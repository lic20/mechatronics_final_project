void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);  // UART communication with computer
  Serial1.begin(115200); // UART communication with XBee module (Pin 18 & 19 of Arduino Mega)
  Serial.println("Begin testing");
  delay(500);

}

void loop() {
  // put your main code here, to run repeatedly:
  // Send data from the serial monitor to Xbee module
  if (Serial.available())
  {
    char outgoing = Serial.read();
    Serial1.print(outgoing);
  }

  // Receive data from the Xbee module and print to serial monitor
  if (Serial1.available())
  {
    char incoming = Serial1.read();
    Serial.print(incoming);
  }
}
