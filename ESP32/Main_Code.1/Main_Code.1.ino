#include <Wire.h>

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


int ballsizex;
int ballsizey;
int ballx;
int bally;

int cgoalsizex;
int cgoalsizey;
int cgoalx;
int cgoaly;

int ygoalsizex;
int ygoalsizey;
int ygoalx;
int ygoaly;





void setup()
{
  Wire.begin();
  delay(100);
  Serial.begin(115200);
  Serial.setTimeout(5);
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
  for (int i = 0; i < 4; i++)
    wDegree[i] = mDegree[i] + 90;
  //Button Mode
  pinMode(4, INPUT_PULLUP);
  button = millis();
}

void loop()
{
  //    turn(1, 100);
  //    turn(2, 100);
  //    turn(3, 100);
  //    turn(4, 100);
  //    dribblerTurn(100);
  //    delay(1000);
  //    turn(1, 0);
  //    turn(2, 0);
  //    turn(3, 0);
  //    turn(4, 0);
  dribblerTurn(60);
  //    delay(500);
  //    turn(1, -100);
  //    turn(2, -100);
  //    turn(3, -100);
  //    turn(4, -100);
  //    delay(1000);
  //turn(5, 100);
  //    delay(1000);
  //testMotor(5);
  //turn(, 100);
  //turn(4, 100);
  //turn(3, 100);
  //turn(4, 100);
  Serial.print(",\tCurrent Sense 1: ");
  Serial.print(((3.3 / 4095.0) * (float)analogRead(36) + 0.14) / 0.1 / 2);
  Serial.print(",\tCurrent Sense 2: ");
  Serial.print(((3.3 / 4095.0) * (float)analogRead(39) + 0.14) / 0.1 / 2);
  Serial.print(",\tCurrent Sense 3: ");
  Serial.print(((3.3 / 4095.0) * (float)analogRead(34) + 0.14) / 0.1 / 2);
  Serial.print(",\tCurrent Sense 4: ");
  Serial.print(((3.3 / 4095.0) * (float)analogRead(32) + 0.14) / 0.1 / 2);
  Serial.print(",\tCurrent Sense 5: ");
  Serial.println(((3.3 / 4095.0) * (float)analogRead(35) + 0.14) / 0.1 / 2);//*

  Button();

  drive(0, -100, 50);
  //  Serial.println();
  Serial.println("data:");
  i2c();
  decode();
}

void i2c()
{
  Wire.requestFrom(0x30, 19);
  while (Wire.available())
  { // slave may send less than requested
    int c = Wire.read(); // receive a byte as character
    Serial.print(c);     // print the character
  }
}

void decode()
{

  String remaining;
  //remaining = "O,148,246,256,78,C,1,2,3,78,Y,45,67,8,9,.";
  remaining = Serial.readStringUntil('.');
  //sizex remaining.substring(remaining.indexOf(),remaining.indexOf()).toInt();
  //while (Serial.available() > 0)
  //{

  if (remaining.substring(0, 1) == "O")
  { //ball
    /*
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
        remaining = remaining.substring(remaining.indexOf(',') + 1);
        Serial.println(remaining.substring(0, remaining.indexOf(',')));
    */

    Serial.println(remaining.substring(0, remaining.indexOf(',')));
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    ballsizex = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    ballsizey = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    ballx = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    bally = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    Serial.println(remaining.substring(0, remaining.indexOf(',')));
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    cgoalsizex = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    cgoalsizey = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    cgoalx = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    cgoaly = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    Serial.println(remaining.substring(0, remaining.indexOf(',')));
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    ygoalsizex = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    ygoalsizey = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    ygoalx = remaining.substring(0, remaining.indexOf(',')).toInt();
    remaining = remaining.substring(remaining.indexOf(',') + 1);
    ygoaly = remaining.substring(0, remaining.indexOf(',')).toInt();

    Serial.println(remaining);



    Serial.println(ballsizex);
    Serial.println(ballsizey);
    Serial.println(ballx);
    Serial.println(bally);
    Serial.println(cgoalsizex);
    Serial.println(cgoalsizey);
    Serial.println(cgoalx);
    Serial.println(cgoaly);
    Serial.println(ygoalsizex);
    Serial.println(ygoalsizey);
    Serial.println(ygoalx);
    Serial.println(ygoaly);

    //x = x + sizex / 2;
    //y = y + sizey / 2;
    //}
  }
}

void testMotor(int num)
{
  for (int i = 0; i > -100; i--)
  {
    turn(num, i);
    delay(50);
  }
  turn(num, 0);
  delay(500);
  for (int i = 0; i < 100; i++)
  {
    turn(num, i);
    delay(50);
  }
  turn(num, 0);
}

void motorSetup(int num, int ina, int inb)
{
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  inAPwm[num - 1] = (num - 1) * 2;
  inBPwm[num - 1] = num * 2 - 1;
  ledcAttachPin(ina, inAPwm[num - 1]);   // assign pins to channels
  ledcSetup(inAPwm[num - 1], freq, res); // 12 kHz PWM, 8-bit resolution
  ledcAttachPin(inb, inBPwm[num - 1]);   // assign pins to channels
  ledcSetup(inBPwm[num - 1], freq, res);
}

void turn(int num, float Pwm)
{
  //  char buf[80];
  //  sprintf(buf, "Motor, %d is running at %d PWM on channels, %d, %d ", num, (int)Pwm, inAPwm[num - 1] , inBPwm[num - 1] );
  //  Serial.print(buf);
  if (Pwm > 0)
  {
    Pwm *= 2.55;
    ledcWrite(inAPwm[num - 1], Pwm);
    ledcWrite(inBPwm[num - 1], 0);
    //    Serial.println("Pwm > 0");
  }
  else if (Pwm < 0)
  {
    Pwm *= -1;
    Pwm *= 2.55;
    //    Serial.print(Pwm);
    ledcWrite(inBPwm[num - 1], (int)Pwm);
    ledcWrite(inAPwm[num - 1], 0);
    //    Serial.print("Pwm < 0");
  }
  else
  {
    ledcWrite(inAPwm[num - 1], 0);
    ledcWrite(inBPwm[num - 1], 0);
    //    Serial.print("else");
  }
}

void drive(float Degrees, int rotate, int Speed)
{

  rotate = abs(rotate) / 100 * Speed;



  int mSpeed[4];
  Degrees -= 90;
  if (Degrees < 0)
    Degrees += 360;

  mSpeed[0] = sin(radians(wDegree[0] - Degrees)) * Speed;
  mSpeed[1] = sin(radians(wDegree[1] - Degrees)) * Speed;
  mSpeed[2] = sin(radians(wDegree[2] - Degrees)) * Speed;
  mSpeed[3] = sin(radians(wDegree[3] - Degrees)) * Speed;


  int Max = mSpeed[0];
  int Min = mSpeed[0];

  for (int i = 0; i < 4; i++)
  {
    if (mSpeed[i] < Min)
      Min = mSpeed[i];
    if (mSpeed[i] > Max)
      Max = mSpeed[i];
  }

  mSpeed[0] = map(mSpeed[0], Min, Max, -Speed, Speed);
  mSpeed[1] = map(mSpeed[1], Min, Max, -Speed, Speed);
  mSpeed[2] = map(mSpeed[2], Min, Max, -Speed, Speed);
  mSpeed[3] = map(mSpeed[3], Min, Max, -Speed, Speed);

  mSpeed[0] *= (Speed - abs(rotate)) / Speed;
  mSpeed[1] *= (Speed - abs(rotate)) / Speed;
  mSpeed[2] *= (Speed - abs(rotate)) / Speed;
  mSpeed[3] *= (Speed - abs(rotate)) / Speed;

  mSpeed[0] += rotate;
  mSpeed[1] += rotate;
  mSpeed[2] += rotate;
  mSpeed[3] += rotate;

  char buf[100];
  sprintf(buf, "M1 Speed, %d M2 Speed, %d M3 Speed, %d M4 Speed, %d", mSpeed[0] , mSpeed[1] , mSpeed[2] , mSpeed[3]);
  Serial.println(buf);
  turn(1, mSpeed[0]);
  Serial.println();
  turn(2, mSpeed[1]);
  Serial.println();
  turn(3, mSpeed[2]);
  Serial.println();
  turn(4, mSpeed[3]);
  Serial.println();
}

void softPwm(int pin, int _freq, float dutyCycle)
{
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
  if (micros() > timer[pin] + timeOff && !onOff[pin])
  {
    timer[pin] = micros();
    digitalWrite(pin, HIGH);
    onOff[pin] = true;
    //Serial.println("ON");
  }
  else if (micros() > (timer[pin]) + timeOn && onOff[pin])
  {
    timer[pin] = micros();
    digitalWrite(pin, LOW);
    onOff[pin] = false;
    //Serial.println("OFF");
  }
  if (dutyCycle == 100)
    digitalWrite(pin, HIGH);
  else if (dutyCycle == 0)
    digitalWrite(pin, LOW);
}

void dribblerTurn(int pwm)
{
  softPwm(dribblerPin, 12000, pwm);
}

void kick()
{
  digitalWrite(kickerPin, HIGH);
  delay(50);
  digitalWrite(kickerPin, LOW);
}

void Button()
{
  if ((button <= millis() - 2500 && digitalRead(4) == LOW) || bypass == true)
  { // Button has been pressed
    if (bypass == false)
    {
      button = millis();
    }
    bypass = true;
    if (button < millis() - 20 && digitalRead(4) == LOW)
    { // Really pressed
      button = millis();
      bypass = false;
      Serial.print("KICK\n");
      kick();
    }
  }
}
