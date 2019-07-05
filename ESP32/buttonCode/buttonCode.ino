byte buttonPin = 14;
uint32_t buttonTimer;
uint32_t buttonTimer2;
uint32_t buttonTime;



void setup() {
  Serial.begin(115200);
  buttonSetup();
}

void loop() {
  button();
//  Serial.println();
}

void button() {

//  Serial.println(digitalRead(buttonPin));

  if (!digitalRead(buttonPin)) {
    buttonTimer  = millis();
//    Serial.print("0");

//    Serial.print("\tbuttonTimer: ");
//    Serial.print(buttonTimer);
//    Serial.print("\tbuttonTimer: ");
//    Serial.print(buttonTimer2);
  }
  else {
//    Serial.print("1");

//    Serial.print("\tbuttonTimer: ");
//    Serial.print(buttonTimer);
//    Serial.print("\tbuttonTimer: ");
//    Serial.print(buttonTimer2);
    if (buttonTimer  > buttonTimer2 ) buttonPressed();
    buttonTimer2  = millis();
  }
}

void buttonSetup() {
  pinMode(buttonPin , INPUT_PULLUP);
}

void buttonPressed() {
  buttonTime  = buttonTimer  - buttonTimer2 ;
  Serial.print("Time: ");
  Serial.println(buttonTime);
  if (buttonTime  > 50 && buttonTime  <= 1000) onOff();
  else if (buttonTime  > 1000 && buttonTime  <= 3000) calibrate();
  else if (buttonTime  > 3000) kick();
}

void onOff() {
  Serial.print("onOff\n");
}

void calibrate() {
  Serial.print("calibrate\n");
}

void kick() {
  Serial.print("kick\n");
}
