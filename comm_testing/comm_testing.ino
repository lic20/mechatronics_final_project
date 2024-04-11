int count = 0;
int match = 0;
void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(115200);  // UART communication with computer
  Serial1.begin(115200); // UART communication with XBee module (Pin 18 & 19 of Arduino Mega)

  while (!Serial) continue;
  while (!Serial1) continue;

  Serial.println("Begin testing");
  delay(500);
}

void loop() {
  get_data();
  delay(5000);
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
  }
  return 0;
}
