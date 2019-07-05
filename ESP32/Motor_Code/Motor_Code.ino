int PWMPin[5];
int InAPin[5];
int InBPin[5];
float mDegree[4] = {50, 130, 230, 310};
float wDegree[4];


void setup() {
  Serial.begin(115200);
  motorSetup(1, 19, 9, 26); //NUM, PWM, INA, INB
  motorSetup(2, 2, 27, 23); //NUM, PWM, INA, INB
  motorSetup(3, 25, 17, 16); //NUM, PWM, INA, INB
  motorSetup(4, 10, 4, 33); //NUM, PWM, INA, INB
  motorSetup(5, 14, 18, 13); //NUM, PWM, INA, INB





  for (int i = 0; i < 4; i ++) wDegree[i] = mDegree[i] + 90;
}

void loop() {
//testMotor(5);
testMotor(3);



  //drive(0, 100, 100);

  Serial.println();
}

void testMotor(int num) {
  for (int i = -100; i < 0; i++) {
    turn(num, i);
    delay(25);
  }
  turn(num, 0);
  delay(500);
  for (int i = 100; i > 0; i--) {
    turn(num, i);
    delay(25);
  }
}

void motorSetup (int num, int pwm, int ina, int inb) {
  PWMPin[num - 1] = pwm;
  InAPin[num - 1] = ina;
  InBPin[num - 1] = inb;
  pinMode(pwm, OUTPUT);
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  ledcAttachPin(pwm, num); // assign pins to channels
  ledcSetup(num, 12000, 8); // 12 kHz PWM, 8-bit resolution
}

void turn(int motorNum, float Pwm) {
  char buf[50];
  sprintf(buf, "Motor, %d is running at %d PWM", motorNum, (int)Pwm);
  Serial.print(buf);
  if (Pwm > 0) {
    Serial.print("Pwm > 0");
    Pwm *= 2.55;
    ledcWrite(motorNum, Pwm);
    digitalWrite(InAPin[motorNum - 1], HIGH);
    digitalWrite(InBPin[motorNum - 1], LOW);
  }
  else if (Pwm < 0) {
    Serial.print("Pwm < 0");
    Pwm *= 2.55;
    ledcWrite(motorNum, -Pwm);
    digitalWrite(InAPin[motorNum - 1], LOW);
    digitalWrite(InBPin[motorNum - 1], HIGH);
  }
  else {
    Serial.print("else");
    ledcWrite(motorNum, 0);
    digitalWrite(InAPin[motorNum - 1], LOW);
    digitalWrite(InBPin[motorNum - 1], LOW);
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

  mSpeed[0] += rotate;
  mSpeed[1] += rotate;
  mSpeed[2] += rotate;
  mSpeed[3] += rotate;
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

  mSpeed[0] += rotate;
  mSpeed[1] += rotate;
  mSpeed[2] += rotate;
  mSpeed[3] += rotate;

  Max = mSpeed[0];
  Min = mSpeed[0];

  for (int i = 0; i < 4; i++) {
    if (mSpeed[i] < Min) Min = mSpeed[i];
    if (mSpeed[i] > Max) Max = mSpeed[i];
  }

  Serial.print(Min);
  Serial.print(Max);

  mSpeed[0] = map(mSpeed[0], Min, Max, -Speed, Speed);
  mSpeed[1] = map(mSpeed[1], Min, Max, -Speed, Speed);
  mSpeed[2] = map(mSpeed[2], Min, Max, -Speed, Speed);
  mSpeed[3] = map(mSpeed[3], Min, Max, -Speed, Speed);

  char buf[100];
  sprintf(buf, "M1 Speed, %d M2 Speed, %d M3 Speed, %d M4 Speed, %d", mSpeed[0] , mSpeed[1] , mSpeed[2] , mSpeed[3]);
  Serial.print(buf);
  turn(1, mSpeed[0]);
  turn(2, mSpeed[1]);
  turn(3, mSpeed[2]);
  turn(4, mSpeed[3]);
}
