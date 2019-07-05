int inAPwm[5];
int inBPwm[5];
int freq = 10000;
int res = 8;
float mDegree[4] = {50, 130, 230, 310};
float wDegree[4];

unsigned long timer[40];
boolean onOff[40];

int dribblerPin = 23;
int kickerPin = 33;


//Buttons
boolean bypass;
unsigned long button;

void setup() {
  Serial.begin(115200);
  //Drive Motors
  motorSetup(1, 19, 18); //NUM, INA, INB
  motorSetup(2, 17, 16); //NUM, INA, INB
  motorSetup(3, 25, 26); //NUM, INA, INB
  motorSetup(4, 27, 12); //NUM, INA, INB
  //Dribbler
  pinMode(dribblerPin, OUTPUT);
  //Current Sense
  pinMode(36, INPUT); //1
  pinMode(39, INPUT); //2
  pinMode(34, INPUT); //3
  pinMode(32, INPUT); //4
  pinMode(35, INPUT); //5
  //Kicker
  pinMode(kickerPin, OUTPUT);

  //Makes omni drive work
  for (int i = 0; i < 4; i ++) wDegree[i] = mDegree[i] + 90;
  //Button Mode
  pinMode(4, INPUT_PULLUP);
  button = millis();


}

void loop() {
    turn(1, 100);
  //  delay(1000);
  //  turn(1, 0);
  //  delay(500);
  //  turn(1, -100);
  //  delay(1000);
  //  turn(5, 100);
  //  delay(500);
  //  testMotor(5);
  //turn(, 100);
//  turn(4, 100);
  //  turn(3, 100);
  //  turn(4, 100);
  //*Serial.print(",\tCurrent Sense 1: ");
//    Serial.print(((3.3 / 4095.0) * (float)analogRead(36) + 0.14) / 0.1 / 2);
//    Serial.print(",\tCurrent Sense 2: ");
//    Serial.print(((3.3 / 4095.0) * (float)analogRead(39) + 0.14) / 0.1 / 2);
//    Serial.print(",\tCurrent Sense 3: ");
//    Serial.print(((3.3 / 4095.0) * (float)analogRead(34) + 0.14) / 0.1 / 2);
//    Serial.print(",\tCurrent Sense 4: ");
//    Serial.print(((3.3 / 4095.0) * (float)analogRead(32) + 0.14) / 0.1 / 2);
//    Serial.print(",\tCurrent Sense 5: ");
//    Serial.println(((3.3 / 4095.0) * (float)analogRead(35) + 0.14) / 0.1 / 2);*/


  Button();
  //  dribblerTurn(100);
  //  drive(0, 0, 100);
  //  Serial.println();
}

void testMotor(int num) {
  for (int i = 0; i > -100; i--) {
    turn(num, i);
    delay(50);
  }
  turn(num, 0);
  delay(500);
  for (int i = 0; i < 100; i++) {
    turn(num, i);
    delay(50);
  }
  turn(num, 0);

}

void motorSetup (int num, int ina, int inb) {
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  inAPwm[num - 1] = (num - 1) * 2;
  inBPwm[num - 1] = num * 2 - 1;
  ledcAttachPin(ina, inAPwm[num - 1]); // assign pins to channels
  ledcSetup(inAPwm[num - 1], freq, res); // 12 kHz PWM, 8-bit resolution
  ledcAttachPin(inb, inBPwm[num - 1]); // assign pins to channels
  ledcSetup(inBPwm[num - 1], freq, res);
}


void turn(int num, float Pwm) {
  //  char buf[80];
  //  sprintf(buf, "Motor, %d is running at %d PWM on channels, %d, %d ", num, (int)Pwm, inAPwm[num - 1] , inBPwm[num - 1] );
  //  Serial.print(buf);
  if (Pwm > 0) {
    Pwm *= 2.55;
    ledcWrite(inAPwm[num - 1], Pwm);
    ledcWrite(inBPwm[num - 1], 0);
    //    Serial.println("Pwm > 0");
  }
  else if (Pwm < 0) {
    Pwm *= -1;
    Pwm *= 2.55;
    //    Serial.print(Pwm);
    ledcWrite(inBPwm[num - 1], (int)Pwm);
    ledcWrite(inAPwm[num - 1], 0);
    //    Serial.print("Pwm < 0");
  }
  else {
    ledcWrite(inAPwm[num - 1], 0);
    ledcWrite(inBPwm[num - 1], 0);
    //    Serial.print("else");
  }
}

void drive(float Degrees, int rotate, int Speed) {
  int mSpeed[4];
  Degrees -= 90;
  if (Degrees < 0) Degrees += 360;

  mSpeed[0] = sin(radians(wDegree[0] - Degrees)) * Speed;
  mSpeed[1] = sin(radians(wDegree[1] - Degrees)) * Speed;
  mSpeed[2] = sin(radians(wDegree[2] - Degrees)) * Speed;
  mSpeed[3] = sin(radians(wDegree[3] - Degrees)) * Speed;

  //  mSpeed[0] += rotate;
  //  mSpeed[1] += rotate;
  //  mSpeed[2] += rotate;
  //  mSpeed[3] += rotate;
  int Max = mSpeed[0];
  int Min = mSpeed[0];

  for (int i = 0; i < 4; i++) {
    if (mSpeed[i] < Min) Min = mSpeed[i];
    if (mSpeed[i] > Max) Max = mSpeed[i];
  }

  mSpeed[0] = map(mSpeed[0], Min, Max, -Speed, Speed);
  mSpeed[1] = map(mSpeed[1], Min, Max, -Speed, Speed);
  mSpeed[2] = map(mSpeed[2], Min, Max, -Speed, Speed);
  mSpeed[3] = map(mSpeed[3], Min, Max, -Speed, Speed);
  //
  //  mSpeed[0] += rotate;
  //  mSpeed[1] += rotate;
  //  mSpeed[2] += rotate;
  //  mSpeed[3] += rotate;
  //
  //  Max = mSpeed[0];
  //  Min = mSpeed[0];
  //
  //  for (int i = 0; i < 4; i++) {
  //    if (mSpeed[i] < Min) Min = mSpeed[i];
  //    if (mSpeed[i] > Max) Max = mSpeed[i];
  //  }
  //
  //  Serial.print(Min);
  //  Serial.print(Max);
  //
  //  mSpeed[0] = map(mSpeed[0], Min, Max, -Speed, Speed);
  //  mSpeed[1] = map(mSpeed[1], Min, Max, -Speed, Speed);
  //  mSpeed[2] = map(mSpeed[2], Min, Max, -Speed, Speed);
  //  mSpeed[3] = map(mSpeed[3], Min, Max, -Speed, Speed);

  //  char buf[100];
  //  sprintf(buf, "M1 Speed, %d M2 Speed, %d M3 Speed, %d M4 Speed, %d", mSpeed[0] , mSpeed[1] , mSpeed[2] , mSpeed[3]);
  //  Serial.println(buf);
  turn(1, mSpeed[0]);
  //  Serial.println();
  turn(2, mSpeed[1]);
  //  Serial.println();
  turn(3, mSpeed[2]);
  //  Serial.println();
  turn(4, mSpeed[3]);
  //  Serial.println();
}

void softPwm(int pin, int _freq, float dutyCycle) {
  long pLength = 1000000 / _freq;
  long timeOn = pLength * (dutyCycle / 100);
  long timeOff = pLength - timeOn;
  /*Serial.print(timeOn);
    Serial.print(",\t");
    Serial.print(timeOff);
    Serial.print(",\t");
    Serial.print(pLength);
    Serial.print(",\t");
    Serial.print(micros() > timer[pin] + timeOff);
    Serial.print(",\t");
    Serial.print(micros());
    Serial.print(",\t");
    Serial.print(timer[pin] + timeOff);

    Serial.print("\n");*/
  if (micros() > timer[pin] + timeOff && !onOff[pin]) {
    timer[pin] = micros();
    digitalWrite(pin, HIGH);
    onOff[pin] = true;
    //Serial.println("ON");
  }
  else if (micros() > (timer[pin]) + timeOn && onOff[pin]) {
    timer[pin] = micros();
    digitalWrite(pin, LOW);
    onOff[pin] = false;
    //Serial.println("OFF");
  }
  if (dutyCycle == 100) digitalWrite(pin, HIGH);
  else if (dutyCycle == 0) digitalWrite(pin, LOW);

}

void dribblerTurn(int pwm) {
  softPwm(dribblerPin, 12000, pwm);
}

void kick() {
  digitalWrite(kickerPin, HIGH);
  delay(50);
  digitalWrite(kickerPin, LOW);
}


void Button() {
  if ((button <= millis() - 2500 && digitalRead(4) == LOW) || bypass == true) {   // Button has been pressed
    if (bypass == false) {
      button = millis();
    }
    bypass = true;
    if (button < millis() - 20 && digitalRead(4) == LOW) { // Really pressed
      button = millis();
      bypass = false;
      Serial.print("KICK\n");
      kick();
    }
  }
}
