#include <Wire.h>
#include <PixyI2C.h>

PixyI2C pixy;

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
int cBSpeed = 100;
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



void setup() {
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
  button();
  i2c();
  pixyCam();
  compass();
  averageCurrent();
  haveBallCalc();
  //  if(millis() % 1000 > 500) drive(90, 0, 40);
  //  else drive(270, 0, 40);
  drive(0, 0, 100);
  //turn(1,100);
  dribblerTurn(100);
  //    Chaseball();

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
  /*
    //Serial.print("x: ");
    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print('\t');
    Serial.print(xR);
    Serial.print('\t');
    Serial.print(yR);*/
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

void drive(float Degrees, int rotate, int Speed) {
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
    if (cTimer1[4] < (millis() - 1000)) { //Time before pwm fall off
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
  softPwm(dribblerPin, 150, pwm); //*motorsOn
}

void pixyCam() {
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
        Serial.println(ballX);
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

void Chaseball() {
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
  if ((current[4] > 1.25 && (current[4] < 2 && !dribblerCurrentLimited)) && (ballX < 170 && ballX > 150)) {
    haveBall = true;
    Serial.print("HAS BALL\n");
  }
  else {
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
}

void kick() {
  Serial.print("kick\n");
  dribblerTurn(0);
  delay(250);
  digitalWrite(kickerPin, HIGH);
  delay(50);
  digitalWrite(kickerPin, LOW);
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
