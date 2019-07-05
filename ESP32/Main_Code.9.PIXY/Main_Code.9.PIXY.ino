#include "BluetoothSerial.h"
#include "EEPROM.h"
#include <Wire.h>
#include <PixyI2C.h>
#include <Average.h>


PixyI2C pixy;

BluetoothSerial SerialBT;

//Motor Variables

int inAPwm[5];
int inBPwm[5];
int freq = 1000;
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
boolean pwmState[40];

int dribblerPin = 23;
int kickerPin = 33;
unsigned long kickerCooldown;

boolean dribblerCurrentLimited = false;


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
float ballXLast;
float ballXRateOfChange;
float ballXLastTime;
float ballXPerTime = 250;
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

//General variables

unsigned long haveBallTimer;
boolean haveBall = false;
int cBSpeed = 30;
enum _goalColour {blue, yellow};
_goalColour goalColour = blue;
enum _communications {com, noCom};
_communications communications = com;
enum _tactics {defult, tactic2}; //REPLACE WITH PROPER NAMES
_tactics tactics = defult;
enum _robotType {attack, defend};
_robotType robotType;
boolean motorsOn = false;




//Buttons/switches
uint8_t buttonPin = 14;
uint32_t buttonTimer;
uint32_t buttonTimer2;
uint32_t buttonTime;

boolean switchState[4];
uint8_t switchPin[4] = {0, 0, 4, 13};

//I2C Bottom Level Data

int colorData[7];
int ultrasonicData[4];
/*
  ///////\\\\\\
  |||Compass|||
  \\\\\\///////
*/
//Offset
float xO = -175.5;
float yO = 253.5;
float zO = -66;
//Raw Data x,y,x
int16_t xR, yR, zR;
//Heading variables
float headingRad, headingDeg, headingCal;
//Final Data Variables
int16_t x, y, z;
//Calibration Variables
float xRadius = 299;
float yRadius = 319;
float zRadius = 250;
float Radius = 250;
Average<float> compassAvg(50);
float averageCalibrationVal;


void setup() {
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Wire.begin();

  compassSetup();

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
  pinMode(currentPin[0], INPUT); //1
  pinMode(currentPin[1], INPUT); //2
  pinMode(currentPin[2], INPUT); //3
  pinMode(currentPin[3], INPUT); //4
  pinMode(currentPin[4], INPUT); //5

  //Kicker
  pinMode(kickerPin, OUTPUT);

  //Makes omni drive work
  for (int i = 0; i < 4; i++) wDegree[i] = mDegree[i] + 90;

  //Buttons
  buttonSetup();
}

void loop() {
  Serial.println("loop");
  button();
  Serial.println("loop");
  dribblerTurn(100);
  Serial.println("loop");
  i2c();
  Serial.println("loop");
  pixyCam();
  compass();
  averageCurrent();
  haveBallCalc();
  //  if(millis() % 1000 > 500) drive(90, 0, 40, 0);
  //  else drive(270, 0, 40, 0);
  Serial.print("goalX: ");
  Serial.print(goalX);
  Serial.print("goalHeight: ");
  Serial.print(goalHeight);
  
  Serial.print("ygoalX: ");
  Serial.print(ygoalX);
  Serial.print("ygoalHeight: ");
  Serial.print(ygoalHeight);
  if (haveBall == true) {
    haveBallLogic();
  }
  if (haveBall == false) {
    chaseBallUpField(); //chaseBallStraight();
  }
  //haveBallLogic();
  Serial.println(haveBall);
  Serial.print("goalX: ");
  Serial.print(goalX);
  Serial.print("goalHeight: ");
  Serial.print(goalHeight);
  Serial.print("ballX: ");
  Serial.print(ballX);
  
  SerialBT.print("goalX: ");
  SerialBT.print(goalX);
  SerialBT.print("goalHeight: ");
  SerialBT.print(goalHeight);
  SerialBT.print("ballX: ");
  SerialBT.print(ballX);
  SerialBT.print("Haveball: ");
  SerialBT.print(haveBall);
  SerialBT.println(); 
  switches();

  /*
      char buf[75];
      sprintf(buf, "\nBallX: %d BallY: %d BallWidth: %d BallHeight: %d", ballX, ballY, ballWidth, ballHeight);
      Serial.print(buf);
  */
  Serial.print(",\tCurrent Sense 1: ");
  Serial.print(current[0]);
  Serial.print(",\tCurrent Sense 2: ");
  Serial.print(current[1]);
  Serial.print(",\tCurrent Sense 3: ");
  Serial.print(current[2]);
  Serial.print(",\tCurrent Sense 4: ");
  Serial.print(current[3]);
  Serial.print(",\tCurrent Sense 5: ");
  Serial.println(current[4]);
}

void i2c() {

  int i = 0;
  Wire.requestFrom(0x30, 13);
  while (Wire.available()) {
    colorData[0] = Wire.read();
    colorData[1] = Wire.read();
    colorData[2] = Wire.read();
    colorData[3] = Wire.read();
    colorData[4] = Wire.read();
    colorData[5] = Wire.read();
    colorData[6] = Wire.read();

    ultrasonicData[0] = Wire.read();
    ultrasonicData[1] = Wire.read();
    ultrasonicData[2] = Wire.read();
    ultrasonicData[3] = Wire.read();

    switchState[0] = Wire.read();
    switchState[1] = Wire.read();
    //    Serial.print(',');
  }

}

void compassSetup() {

  EEPROM.begin(64);
  averageCalibrationVal = EEPROM.readFloat(0);

  Wire.beginTransmission(0x1E); //open communication with HMC5883
  Wire.write(0x02);             //select mode register
  Wire.write(0x00);             //continuous measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(0x1E); //open communication with HMC5883
  Wire.write(0x01);             //select configoration register B
  Wire.write(0x01);             //scale 1.3
  Wire.endTransmission();


}

void compass() {
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

  x = (xR - (int)xO) * (Radius * 2 / xRadius);
  y = (yR - (int)yO) * (Radius * 2 / yRadius);
  z = (zR - (int)zO) * (Radius * 2 / yRadius);

  headingRad = atan2(y, x); //convects from x,y to radians.
  if (headingRad < 0)
    headingRad += 2 * PI;
  // Check for wrap due to addition of declination.
  if (headingRad > 2 * PI)
    headingRad -= 2 * PI;
  // Convert radians to degrees for readability.
  headingDeg = headingRad * 180 / PI;
  //Print out values of each axis
  /*
    //Serial.print("x: ");
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print('\t');
    Serial.print(xR);
    Serial.print('\t');
    Serial.print(yR);*/
  calculateCompass();


}

void testMotor(int num) {
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

void motorSetup(int num, int ina, int inb) {
  pinMode(ina, OUTPUT);
  pinMode(inb, OUTPUT);
  inAPwm[num - 1] = (num - 1) * 2;
  inBPwm[num - 1] = num * 2 - 1;
  ledcAttachPin(ina, inAPwm[num - 1]);   // assign pins to channels
  ledcSetup(inAPwm[num - 1], freq, res); // 12 kHz PWM, 8-bit resolution
  ledcAttachPin(inb, inBPwm[num - 1]);   // assign pins to channels
  ledcSetup(inBPwm[num - 1], freq, res);
}

void turn(int num, float Pwm) {
  Pwm *= motorsOn;
  //  char buf[80];
  //  sprintf(buf, "Motor, %d is running at %d PWM on channels, %d, %d ", num, (int)Pwm, inAPwm[num - 1] , inBPwm[num - 1] );
  //  Serial.print(buf);
  if (current[num - 1] >= 1.5) { // amps before pwm fall off
    if (cTimer1[num - 1] < (millis() - 1000)) { //Time before pwm fall off
      if (pwmLimit[num - 1] > 20 && cTimer2[num - 1] < (millis() - 50)) { // min pwm percent
        cTimer2[num - 1] = millis();
        pwmLimit[num - 1] -= 5; // Pwm decrement per time interval
      }
    }
  }
  else if (pwmLimit[num - 1] != 100) { // if pwm is not at 100%
    if (pwmLimit[num - 1] < 100 && cTimer2[num - 1] < (millis() - 50)) { // increment until pwm is at or above 100% each time interval
      cTimer2[num - 1] = millis();
      pwmLimit[num - 1] += 10; // Pwm increment per time interval
    }
    if (pwmLimit[num - 1] > 100)
    {
      pwmLimit[num - 1] = 100; //set pwm to 100% if is above 100%
    }
  }
  else
  {
    cTimer1[num - 1] = millis(); //reset timer if pwm is at 100% and the current is less than limited amount.
  }
  /*
    Serial.print("PWMLIMIT");
    Serial.print(num);
    Serial.print(":\t");
    Serial.print(pwmLimit[num - 1]);
    Serial.print("\t");
  */

  if (pwmLimit[num - 1] < abs(Pwm) && millis() > 1000) {//Doesn't limit in first 1 second due to current going stupid, reduces pwm only if its less limited pwm
    if (Pwm < 0) Pwm = -pwmLimit[num - 1];
    else if (Pwm > 0) Pwm = pwmLimit[num - 1];
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

void drive(float Degrees, float targetHeading, int Speed, int rotateAmount) {
  int mSpeed[4];
  float rotate;
  float maxRotateSpeed = 99;
  if (Speed > 10) {
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

    float useHeading = headingCal + targetHeading;

    if (useHeading < -180) useHeading += 360;
    if (useHeading > 180) useHeading -= 360;

    if (0 > useHeading) rotate = map(useHeading, -180, 0, maxRotateSpeed, 0);
    else if (useHeading > 0) rotate = map(useHeading, 180, 0, -maxRotateSpeed, 0);
    else rotate = 0;

    if (rotateAmount != 0) rotate = rotateAmount;

    mSpeed[0] *= (100 - abs(rotate)) / 100;
    mSpeed[1] *= (100 - abs(rotate)) / 100;
    mSpeed[2] *= (100 - abs(rotate)) / 100;
    mSpeed[3] *= (100 - abs(rotate)) / 100;

    mSpeed[0] += rotate / 100 * Speed;
    mSpeed[1] += rotate / 100 * Speed;
    mSpeed[2] += rotate / 100 * Speed;
    mSpeed[3] += rotate / 100 * Speed;
  }
  else {
    mSpeed[0] = 0;
    mSpeed[1] = 0;
    mSpeed[2] = 0;
    mSpeed[3] = 0;
  }

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

void softPwm(int pin, int _freq, float dutyCycle) {
  long pLength = 1000000 / _freq;
  long timeOn = pLength * (dutyCycle / 100);
  long timeOff = pLength - timeOn;
  /*
    Serial.print(timeOn);
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
    Serial.println();
  */
  if (micros() > timer[pin] + timeOff && !pwmState[pin])
  {
    timer[pin] = micros();
    digitalWrite(pin, HIGH);
    pwmState[pin] = true;
    //Serial.println("ON");
  }
  else if (micros() > (timer[pin]) + timeOn && pwmState[pin])
  {
    timer[pin] = micros();
    digitalWrite(pin, LOW);
    pwmState[pin] = false;
    //Serial.println("OFF");
  }
  if (dutyCycle == 100)
    digitalWrite(pin, HIGH);
  else if (dutyCycle == 0)
    digitalWrite(pin, LOW);
}

void dribblerTurn(int pwm) {
  if (current[4] >= 2) { // amps before pwm fall off
    if (cTimer1[4] < (millis() - 0)) { //Time before pwm fall off
      if (pwmLimit[4] > 20 && cTimer2[4] < (millis() - 50)) { // min pwm percent
        cTimer2[4] = millis();
        pwmLimit[4] -= 5; // Pwm decrement per time interval
        dribblerCurrentLimited = true;
      }
    }
  }
  else if (pwmLimit[4] != 100) { // if pwm is not at 100%
    if (pwmLimit[4] < 100 && cTimer2[4] < (millis() - 50)) { // increment until pwm is at or above 100% each time interval
      cTimer2[4] = millis();
      pwmLimit[4] += 10; // Pwm increment per time interval
    }
    if (pwmLimit[4] > 100)
    {
      pwmLimit[4] = 100; //set pwm to 100% if is above 100%
    }
  }
  else
  {
    cTimer1[4] = millis(); //reset timer if pwm is at 100% and the current is less than limited amount.
    dribblerCurrentLimited = false;
  }
  if (pwmLimit[4] < pwm && millis() > 1000) {//Doesn't limit in first 1 second due to current going stupid, reduces pwm only if its less limited pwm
    pwm = pwmLimit[4];
  }
  softPwm(dribblerPin, 150, pwm * motorsOn); //
}

void pixyCam() {
  int i = 0;
  Wire.requestFrom(0x01, 12);
  while (Wire.available()) {
    ballX = Wire.read();
    ballY = Wire.read();
    ballHeight = Wire.read();
    ballWidth = Wire.read();
    ygoalX = Wire.read();
    ygoalY = Wire.read();
    ygoalHeight = Wire.read();
    ygoalWidth = Wire.read();
    bgoalX = Wire.read();
    bgoalY = Wire.read();
    bgoalHeight = Wire.read();
    bgoalWidth = Wire.read();
  }
    Serial.print("ballX: ");
    Serial.print(ballX);
    Serial.print("ballY: ");
    Serial.print(ballY);
    Serial.print("ballHeight: ");
    Serial.print(ballHeight);
    Serial.print("ballWidth: ");
    Serial.print(ballWidth);
    Serial.print("ygoalX: ");
    Serial.print(ygoalX);
    Serial.print("ygoalY: ");
    Serial.print(ygoalY);
    Serial.print("ygoalHeight: ");
    Serial.print(ygoalHeight);
    Serial.print("ygoalWidth: ");
    Serial.print(ygoalWidth);
    Serial.print("bgoalX: ");
    Serial.print(bgoalX);
    Serial.print("bgoalY: ");
    Serial.print(bgoalY);
    Serial.print("bgoalHeight: ");
    Serial.print(bgoalHeight);
    Serial.print("bgoalWidth: ");
    Serial.print(bgoalWidth);

    Serial.println();
  
        ballX += 5;
        if (ballX > 320) ballX = 320;

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
  lastBallX = ballX;
  lastygoalX = ygoalX;
  lastbgoalX = bgoalX;

  if (goalColour == blue) {
    Serial.print("Blue");
    goalY = bgoalY;
    goalX = bgoalX;
    goalHeight = bgoalHeight;
  }
  else if (goalColour == yellow) {
    Serial.print("Yellow");
    goalY = ygoalY;
    goalX = ygoalX;
    goalHeight = ygoalHeight;
  }
}

void chaseBallStraight() { //LITERALLY TURNS TO BALL AND DRIVE FORWARDS
  Serial.print("ChaseBall: ");
  int correctionSpeed;

  if (ballX > 160) correctionSpeed = -map(ballX - 160, 0, 160, 0, 100);
  if (ballX < 160) correctionSpeed = map(160 - ballX, 0, 160, 0, 100);
  if (ballX < 160 && ballX > 140) correctionSpeed = 0;

  drive(0, 0, cBSpeed, correctionSpeed);
}

void chaseBallUpField() {
  Serial.print("ChaseBall: ");
  if (ballX == 320 || ballX == 0) drive(180, 0, cBSpeed, 0); //Go back if can't see the ball
  else if (ballX > 280) drive(-130, 0 , cBSpeed, 0); //if it is on the left edge of screen go back left
  else if (ballX < 40) drive(130, 0 , cBSpeed, 0); //if it is on the right edge of screen go back right
  else if (ballX + ballXRateOfChange < 163 && ballX - ballXRateOfChange > 126) drive(0, 0, cBSpeed, 0); //Go forwards if the ball is in from in x time period
  else if (ballX < 163 && ballX > 126) drive(0, 0, cBSpeed, 0); //Go forwards if the ball is in front
  else if (ballX > 160) { // if on left and everything else is not true
    if (ballY < 120) drive(-90, 0, cBSpeed, 0); //Go left if the ball is to the left
    else drive(-50, 0, cBSpeed, 0); //Go forwards left if it is not too close to the ball
  }
  else if (ballX < 160) { // if on right and everything else is not true
    if (ballY < 120) drive(90, 0, cBSpeed, 0); //Go right if the ball is to the right
    else drive(50, 0, cBSpeed, 0); //Go forwards right if it is not too close to the ball
  }
  else drive(0, 0, 0, 0); //Should Never Get Here

  ballXRateOfChange = ballXLast - ballX;
  if (ballXRateOfChange > 1 || ballXRateOfChange < -1) ballXRateOfChange *=  (ballXPerTime / float(millis() - ballXLastTime));
  else ballXRateOfChange = 0;
  ballXLast = ballX;
  ballXLastTime = millis();
  
}

void haveBallLogic() {
  int correctionSpeed;
  Serial.print("Haveballlogic: ");


  if (goalX > 140 && goalX < 180) {
    Serial.println("Forwards");
    correctionSpeed = 0;
    if (goalHeight > 35) {
//      if (motorsOn) kick();
    }
  }
  else if (goalX > 160) {
    Serial.println("CorrectLeft");
    correctionSpeed = -map(goalX - 160, 0, 160, 0, 100); //-map(goalX - 160, 0, 160, 0, 100);
  }
  else if (goalX < 160) {
    Serial.println("CorrectRight");
    correctionSpeed = map(160 - goalX, 0, 160, 0, 100); //map(160 - goalX, 0, 160, 0, 100);
  }

  if (correctionSpeed != 0) drive(0, 0, cBSpeed, correctionSpeed);
  else drive(0, 0, cBSpeed, 1);




}

void averageCurrent() {
  char output[50];
  for (int j = 0; j < 5; j++) {
    //    Serial.print("Current ");
    //    Serial.print(j);
    //    Serial.print(":\t");
    //    sprintf(output, "Size of current: %d", sizeof(avgCurrent));
    //    Serial.println(output);
    for (int i = ((sizeof(avgCurrent) / 4 ) / 5) - 1; i > 0; i--) {
      //      sprintf(output, "Before Val %d is %d and %d", i + 1, (int)avgCurrent[i][j], (int)avgCurrent[i - 1][j]);
      //      Serial.println(output);
      avgCurrent[i][j] = avgCurrent[i - 1][j];
      //      sprintf(output, "After Val %d is %d and %d", i + 1, (int)avgCurrent[i][j], (int)avgCurrent[i - 1][j]);
      //      Serial.println(output);
    }
    avgCurrent[0][j] = ((3.3 / 4095.0) * (float)analogRead(currentPin[j]) + 0.14) / 0.1 / 2;
    //    sprintf(output, "Val %d is %d", 1, (int)avgCurrent[0][j]);
    //    Serial.println(output);
    for (int i = 0; i < ((sizeof(avgCurrent) / 4 ) / 5) - 1; i++) {
      current[j] += avgCurrent[i][j];
    }
    current[j] /= ((sizeof(avgCurrent) / 4 ) / 5);

    //    Serial.print("Amps: ");
    //    Serial.print(current[j], 2);
  }
}

void haveBallCalc() {
  if ((current[4] > 1.25 || dribblerCurrentLimited) && (ballX < 163 && ballX > 126)) {
    Serial.print(current[4] > 1.25);
    Serial.print(!dribblerCurrentLimited);
    Serial.print(ballX < 163 && ballX > 126);

    haveBall = true;
    Serial.print("HAS BALL\n");
    haveBallTimer = millis();
  }
  else if (haveBallTimer + 500 < millis()){
    haveBall = false;
    Serial.print("DOES NOT HAVE BALL\n");
  }
}

void button() {
  if (!digitalRead(buttonPin)) {
    buttonTimer  = millis();
  }
  else {
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
  if (motorsOn) motorsOn = false;
  else motorsOn = true;

}

void calibrate() {
  Serial.print("calibrate\n");
  compassAvg.clear();
  for (int i = 0; i < 50; i++) {
    compass();
    compassAvg.push(headingDeg);
    delay(50);
  }
  averageCalibrationVal = compassAvg.mean();
  //  Serial.print(averageCalibrationVal);

  EEPROM.writeFloat(0, averageCalibrationVal);
  EEPROM.commit();
}

void kick() {
  Serial.print("kick\n");
  if (kickerCooldown + 3000 < millis()) {
    Serial.print("AcuallyKicked\n");
    drive(0, 0, cBSpeed, 1);
    kickerCooldown = millis();
    dribblerTurn(0);
    delay(250);
    digitalWrite(kickerPin, HIGH);
    delay(50);
    digitalWrite(kickerPin, LOW);
  }
}

void switches () {
  switchState[2] = digitalRead(switchPin[2]);
  switchState[3] = digitalRead(switchPin[3]);

  if (switchState[0] == 1) goalColour = yellow;
  else goalColour = blue;

  if (switchState[1] == 1) communications = noCom;
  else communications = com;

  if (switchState[2] == 1) tactics = tactic2;
  else tactics = defult;

  if (switchState[3] == 1) robotType = defend;
  else robotType = attack;

  char buf[150];
  sprintf(buf, "\ngoalColour: %d \ncommunications: %d \ntatics: %d \nrobotType: %d", goalColour, communications, tactics, robotType);
  Serial.print(buf);
}

void calculateCompass () {
  headingCal = headingDeg - averageCalibrationVal;
  if (headingCal < 0) headingCal += 360;
  if (headingCal > 180) headingCal -= 360;
  Serial.print("Calibrated Compass Value: ");
  Serial.print(headingCal);
}
