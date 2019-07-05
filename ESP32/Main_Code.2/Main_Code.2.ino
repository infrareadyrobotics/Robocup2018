#include <Wire.h>
#include <PixyI2C.h>

PixyI2C pixy;

int inAPwm[5];
int inBPwm[5];
int freq = 10000;
int res = 8;
float mDegree[4] = {50, 130, 230, 310};
float wDegree[4];
float current[5];
byte currentPin[5] = {36, 39, 34, 32, 35};
unsigned long cTimer1[5];
unsigned long cTimer2[5];
int pwmLimit[5];
float avgCurrent[8][5];


unsigned long timer[40];
boolean onOff[40];

int dribblerPin = 23;
int kickerPin = 33;

//Pixy Variables

int lastBallX;
int lastygoalX;
int lastbgoalX;
int pixyTimer[4];
int pixySigniture;
int pixyX;
int pixyY;
int pixyWidth;
int pixyHeight;

int ballX;
int ballY;
int ballWidth;
int ballHeight;
// float ballPos;
// float ballDist;
// float distA = -130.893900019;
// float distB = 1.96648958168;
// float distC = -26.5671704141;
// float goalPos;
// float goalDist;
// float goalDistAvg[10];
int goalY;
int goalHeight;
// float lastBallDist;
boolean canSeeBall = true;
int goalX;

int bgoalX;
int bgoalY;
int bgoalWidth;
int bgoalHeight;

int ygoalX;
int ygoalY;
int ygoalWidth;
int ygoalHeight;

int cBSpeed = 100;

//Buttons
boolean bypass;
unsigned long button;

int colorData[7];
int ultrasonicData[4];

//Offset
float xO = 45;
float yO = -56.5;
float zO = -66;
//Raw Data x,y,x
int16_t xR, yR, zR;
//Heading variables
float headingRad, headingDeg;
//Final Data Variables
int16_t x, y, z;
//Calibration Variables
float xRadius = 113.5;
float yRadius = 97.5;
float zRadius = 100;
float Radius = 100;

void setup()
{
  Wire.begin();
  Wire.beginTransmission(0x1E); //open communication with HMC5883
  Wire.write(0x02);             //select mode register
  Wire.write(0x00);             //continuous measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(0x1E); //open communication with HMC5883
  Wire.write(0x01);             //select configoration register B
  Wire.write(0x01);             //scale 1.3
  Wire.endTransmission();

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
  //dribblerTurn(60);
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
  currentMeasure();
  averageCurrent();
  Serial.print(",\tCurrent Sense 1: ");
  Serial.print(current[0]);
  Serial.print(",\tCurrent Sense 2: ");
  Serial.print(current[1]);
  Serial.print(",\tCurrent Sense 3: ");
  Serial.print(current[2]);
  Serial.print(",\tCurrent Sense 4: ");
  Serial.print(current[3]);
  Serial.print(",\tCurrent Sense 5: ");
  Serial.println(current[4]); //*

  Button();

  //drive(0, 0, 50);
  //drive(0, 0, 100);
  //  Serial.println();
  compass();
  Serial.println("data:");
  i2c();
  pixyCam();
  dribblerTurn(100);
  Chaseball();

  // char buf[100];
  // sprintf(buf, "\nBallX: %d BallY: %d BallWidth: %d BallHeight: %d", ballX, ballY, ballWidth, ballHeight);
  // Serial.print(buf);
}

void i2c()
{
  int i;
  Wire.requestFrom(0x30, 10);
  while (Wire.available())
  {
    i++;
    if (i <= 6)
    {
      colorData[i] = Wire.read();
      Serial.print(colorData[i]);
    }
    else
    {
      ultrasonicData[i - 6] = Wire.read();
      Serial.print(ultrasonicData[i - 6]);
    }

    Serial.print(',');
  }
}

void compass()
{
  Wire.beginTransmission(0x1E);
  Wire.write(0x03); //select register 3, X MSB register AKA as the data registor
  Wire.endTransmission();

  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(0x1E, 6);
  if (6 <= Wire.available())
  {
    xR = Wire.read() << 8; //X msb
    xR |= Wire.read();     //X lsb
    zR = Wire.read() << 8; //Z msb
    zR |= Wire.read();     //Z lsb
    yR = Wire.read() << 8; //Y msb
    yR |= Wire.read();     //Y lsb
  }

  x = (xR - (int)xO) * (Radius / xRadius);
  y = yR - (int)yO * (Radius / yRadius);
  z = zR - (int)zO * (Radius / yRadius);

  headingRad = atan2(y, x); //convects from x,y to radians.
  if (headingRad < 0)
    headingRad += 2 * PI;
  // Check for wrap due to addition of declination.
  if (headingRad > 2 * PI)
    headingRad -= 2 * PI;
  // Convert radians to degrees for readability.
  headingDeg = headingRad * 180 / PI;
  //Print out values of each axis

  //Serial.print("x: ");
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print('\t');
  Serial.print(xR);
  Serial.print('\t');
  Serial.print(yR);
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
  if (current[num-1] >= 1) { // amps before pwm fall off
    if (cTimer1[num-1] < (millis() - 1000)) { //Time before pwm fall off
      if (pwmLimit[num-1] > 20 && cTimer2[num-1] < (millis() - 50)) { // min pwm percent
        cTimer2[num-1] = millis();
        pwmLimit[num-1] -= 5; // Pwm decrement per time interval
      }
    }
  }
  else if (pwmLimit[num-1] != 100){ // if pwm is not at 100%
    if (pwmLimit[num-1] < 100 && cTimer2[num-1] < (millis() - 50)) { // increment until pwm is at or above 100% each time interval
      cTimer2[num-1] = millis();
      pwmLimit[num-1] += 10; // Pwm increment per time interval
    }
    if (pwmLimit[num-1] > 100)
    {
      pwmLimit[num-1] = 100; //set pwm to 100% if is above 100%
    }
  }
  else
  {
    cTimer1[num-1] = millis(); //reset timer if pwm is at 100% and the current is less than limited amount.
  }

  Serial.print("PWMLIMIT");
  Serial.print(num);
  Serial.print(":\t");
  Serial.print(pwmLimit[num-1]);
  Serial.print("\t");


  if(pwmLimit[num-1] < abs(Pwm)){
    if (Pwm < 0) Pwm = -pwmLimit[num-1];
    else if (Pwm > 0) Pwm = pwmLimit[num-1];
  }

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
    ledcWrite(inBPwm[num - 1], Pwm);
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
  sprintf(buf, "M1 Speed, %d M2 Speed, %d M3 Speed, %d M4 Speed, %d", mSpeed[0], mSpeed[1], mSpeed[2], mSpeed[3]);
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

void pixyCam()
{
  //Serial.print("Pixy:");
  uint8_t blocks;
  blocks = pixy.getBlocks();
  lastBallX = ballX;
  lastygoalX = ygoalX;
  lastbgoalX = bgoalX;
  if (blocks)
  {
    for (int q = blocks - 1; q >= 0; q--)
    {
      pixyX = pixy.blocks[q].x;
      pixyY = pixy.blocks[q].y;
      pixyWidth = pixy.blocks[q].width;
      pixyHeight = pixy.blocks[q].height;
      pixySigniture = pixy.blocks[q].signature;
      if (pixySigniture == 1)
      {
        pixyTimer[0] = 0;
        ballX = pixyX;
        ballY = pixyY;
        ballWidth = pixyWidth;
        ballHeight = pixyHeight;
      }
      if (pixySigniture == 2)
      {
        pixyTimer[1] = 0;
        bgoalX = pixyX;
        bgoalY = pixyY;
        bgoalWidth = pixyWidth;
        bgoalHeight = pixyHeight;
      }
      if (pixySigniture == 3)
      {
        pixyTimer[2] = 0;
        ygoalX = pixyX;
        ygoalY = pixyY;
        ygoalWidth = pixyWidth;
        ygoalHeight = pixyHeight;
      }
    }
  }
  else if (pixyTimer[0] >= 10)
  {
    if (ballX > 160)
    {
      ballX = 320;
    }
    if (ballX < 160)
    {
      ballX = 0;
    }
  }
  else if (ballX == lastBallX)
  {
    pixyTimer[0]++;
  }
  else if (pixyTimer[1] >= 10)
  {
    if (bgoalX > 160)
    {
      bgoalX = 320;
    }
    if (bgoalX < 160)
    {
      bgoalX = 0;
    }
  }
  else if (bgoalX == lastbgoalX)
  {
    pixyTimer[1]++;
  }

  else if (pixyTimer[2] >= 10)
  {
    if (ygoalX > 160)
    {
      ygoalX = 320;
    }
    if (ygoalX < 160)
    {
      ygoalX = 0;
    }
  }
  else if (ygoalX == lastygoalX)
  {
    pixyTimer[2]++;
  }
}

void Chaseball()
{
  Serial.print("ChaseBall: ");
  //1 = Forward
  //2 = Forward Left
  //3 = Forward Right
  //4 = Back Left
  //5 = Back Right
  //6 = Back
  if (ballX < 195 && ballX > 125)
  { //
    drive(0, 0, cBSpeed);
    Serial.print("ballX < 195 && ballX > 125");
  }
  else if (ballX == 320 || ballX == 0)
  {
    Serial.print("ballX == 320 || ballX == 0");
    drive(180, 0, cBSpeed);
  }
  else if (ballX > 160)
  { //Close to left edge
    Serial.print("ballX > 160");
    if (ballY > 60)
    { // makes sure the ball is not too close
      drive(-50, 0, cBSpeed);
    }
    else
    {
      drive(-130, 0, cBSpeed);
    }
  }
  else if (ballX < 160)
  { //Close to right edge
    Serial.print("ballX < 160");
    if (ballY > 60)
    {
      drive(50, 0, cBSpeed);
    }
    else
    {
      drive(130, 0, cBSpeed);
    }
  }
  else
  {
    drive(0, 0, 0);
    Serial.print("StopShouldNeverGetHere");
  }
}

void currentMeasure()
{
  for (int i = 0; i < 5; i++)
    current[i] = ((3.3 / 4095.0) * (float)analogRead(currentPin[i]) + 0.14) / 0.1 / 2;
}

void averageCurrent() {
  char output[50];
  for(int j = 0; j < 5; j++){
    Serial.print("Current ");
    Serial.print(j);
    Serial.print(":\t");
    sprintf(output, "Size of current: %d", sizeof(avgCurrent));
    Serial.println(output);
    for (int i = ((sizeof(avgCurrent) / 4 ) / 5) - 1; i > 0 / 4; i--) {
      avgCurrent[i][j] = avgCurrent[i - 1][j];
      sprintf(output, "Val %d is %d", i + 1, (int)avgCurrent[i][j]);
      Serial.println(output);
    }
    avgCurrent[0][j] = analogRead(currentPin[j]);
    sprintf(output, "Val %d is %d", 1, (int)avgCurrent[0]);
    Serial.println(output);
    for (int i = 0; i < ((sizeof(avgCurrent) / 4 ) / 5) -1; i++) {
      current[j] += avgCurrent[i][j];
    }
    current[j] /= ((sizeof(avgCurrent) / 4 ) / 5);
    sprintf(output, "Amps: %d", (int)current[j]);
    Serial.println(output);
  } 
}