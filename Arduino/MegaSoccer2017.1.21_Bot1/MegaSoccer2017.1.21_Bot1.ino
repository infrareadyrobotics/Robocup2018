#include <EEPROMex.h>
#include <EEPROMVar.h>
#include <Average.h>
#include <EnableInterrupt.h> //enables lots of extra interupt pins
//Compass from MagMaster
#include <Wire.h>
#include <HMC5883L.h>
#include <PixyI2C.h>

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

PixyI2C pixy;

enum colour {yellow, blue};
colour goalColour = yellow;

enum state {attack,defend};
state robotType = defend;

int lastBallX;
int lastygoalX;
int lastbgoalX;
int pixyTimer[3];
int pixySigniture;
int pixyX;
int pixyY;
int pixyWidth;
int pixyHeight;
int t;

int ballX;
int ballY;
int ballWidth;
int ballHeight;
int ballPos;
int ballDist;
float distA = -130.893900019;
float distB = 1.96648958168;
float distC = -26.5671704141;
int goalPos;

int bgoalX;
int bgoalY;
int bgoalWidth;
int bgoalHeight;

int ygoalX;
int ygoalY;
int ygoalWidth;
int ygoalHeight;

int xv, yv, zv;
volatile float headingDegrees;
float heading;
float heading_last;
float last_calibrated_values[3];

//calibrated_values[3] is the global array where the calibrated data will be placed
//calibrated_values[3]: [0]=Xc, [1]=Yc, [2]=Zc
volatile float calibrated_values[3];
//transformation(float uncalibrated_values[3]) is the function of the magnetometer data correction
//uncalibrated_values[3] is the array of the non calibrated magnetometer data
//uncalibrated_values[3]: [0]=Xnc, [1]=Ync, [2]=Znc

void transformation(float uncalibrated_values[3]) {
  //calibration_matrix[3][3] is the transformation matrix
  //replace M11, M12,..,M33 with your transformation matrix data
  double calibration_matrix[3][3] =
  {
    {1.069, 0.033, 0.032}, //M11-M13
    {0.019, 1.037, 0.011}, //M21-M23
    {0.021, 0.009, 1.118} //M31-M33
  };
  //bias[3] is the bias
  //replace Bx, By, Bz with your bias data
  double bias[3] =
  {
    77.182,//X
    -75.096,//Y
    -326.966//Z
  };
  //calculation
  for (int i = 0; i < 3; ++i) uncalibrated_values[i] = uncalibrated_values[i] - bias[i]; //Takes the appropiate bias of the compassval
  float result[3] = {0, 0, 0}; //resets data
  for (int i = 0; i < 3; ++i) // making loop for MX1-MX3 (Horizontal)
    for (int j = 0; j < 3; ++j) // making loop for vertical
      result[i] += calibration_matrix[i][j] * uncalibrated_values[j]; //times
  for (int i = 0; i < 3; ++i) calibrated_values[i] = result[i];
}

//vector_length_stabilasation() - is the function of the magnetometer vector length stabilasation (stabilisation of the sphere radius)
float scaler;
boolean scaler_flag = false;
float normal_vector_length;
void vector_length_stabilasation() {
  //calculate the normal vector length
  if (scaler_flag == false)
  {
    getHeading();
    normal_vector_length = sqrt(calibrated_values[0] * calibrated_values[0] + calibrated_values[1] * calibrated_values[1] + calibrated_values[2] * calibrated_values[2]);
    scaler_flag = true;
  }
  //calculate the current scaler
  scaler = normal_vector_length / sqrt(calibrated_values[0] * calibrated_values[0] + calibrated_values[1] * calibrated_values[1] + calibrated_values[2] * calibrated_values[2]);
  //apply the current scaler to the calibrated coordinates (global array calibrated_values)
  calibrated_values[0] = calibrated_values[0] * scaler;
  calibrated_values[1] = calibrated_values[1] * scaler;
  calibrated_values[2] = calibrated_values[2] * scaler;
}

unsigned long kickerForwardTimer;
unsigned long kickerCooldown;
boolean kickerForwardReset = true;
int kickerForwardsCount;
boolean canKick;

boolean DisableMode = true;

unsigned long I2CComCooldown;
volatile unsigned long temptimer;
int Switch;
//Motors Varaibles
volatile byte correctAmountMultiplier;
byte correctAmount;
boolean CorrectRight;
boolean CorrectLeft;
boolean Straight;
unsigned long button[2];
boolean motorsOn = false;
byte kicker = 0;
boolean bypass[2];
Average<float> compassCalibrate(100);
unsigned long compassCalibrateTotal;
float compassCalibratedVal;

int directionSend;
uint8_t Direction;

//haveBall
boolean haveBall = false;


//Compass Varibles Working
boolean runOnce = LOW;
volatile float headinDegrees;
float startCompassReading;
float calibratedCompassValue;
float compassValueCalc;
//Motor Comunication Varibles
byte Identifier;

unsigned long haveBallTime = micros();

void setup() {
  Serial.begin(250000);
  pixy.init();
  //Setup Compass
  Wire.begin();
  setupHMC5883L();
  //Buttons and jumpers
  pinMode(22, INPUT_PULLUP); //Turning motors on and off
  pinMode(24, INPUT_PULLUP); //button for Kicker + Compass Calibration
  pinMode(23, INPUT_PULLUP); // Jumpers
  pinMode(25, INPUT_PULLUP);
  pinMode(27, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(35, OUTPUT);
  pinMode(37, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(36, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(48, OUTPUT);
  digitalWrite(27, LOW);
  digitalWrite(29, LOW);
  digitalWrite(31, LOW);
  digitalWrite(33, LOW);
  digitalWrite(35, LOW);
  digitalWrite(37, LOW);
  digitalWrite(A6, LOW);
  digitalWrite(A5, LOW);
  digitalWrite(A4, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A2, LOW);
  digitalWrite(26, LOW);
  digitalWrite(28, LOW);
  digitalWrite(30, LOW);
  digitalWrite(32, LOW);
  digitalWrite(34, LOW);
  digitalWrite(36, LOW);
  digitalWrite(38, LOW);
  digitalWrite(40, LOW);
  digitalWrite(42, LOW);
  digitalWrite(44, LOW);
  digitalWrite(46, LOW);
  digitalWrite(48, LOW);


  //Set direction correction to LOW
  CorrectRight = LOW;
  CorrectLeft = LOW;
  Straight = HIGH;

  startCompassReading = EEPROM.readFloat(1);

}

void loop() {
  ballPos = ballY - ballHeight/2;
  ballDist = distC * tan(ballPos/distA)+distB;
  Serial.print("ballDist:\t");
  Serial.print(ballDist); 
  pixyCam();
  disableMotorsButton();
  calibrationKickerButton();
  GetHeadingDegrees();
  float compassValue = headingDegrees;
  compassValueCalc = compassValue;
  compassCalibration();
  CorrectDirection();
  haveBallCalc();
  //haveBall = true; //Forcefully turns haveball on
  if (haveBall == true) haveBallLogic();
  else Chaseball();
  //abcdefg123Serial.print(" HaveBall: ");
  //abcdefg123Serial.print(haveBall);
  //abcdefg123Serial.print(" ");*/
  //abcdefg123Serial.print("\tstartCompassReading: ");
  //abcdefg123Serial.print(startCompassReading);
  //abcdefg123Serial.print("\tcalibratedCompassValue: ");
  //abcdefg123Serial.print(calibratedCompassValue);
  //abcdefg123Serial.print("UltraSonicsValue: ");
  //abcdefg123Serial.print(frontUltraSonic+backUltraSonic);
  //abcdefg123Serial.print("\tcompassValueCalc: ");
  //abcdefg123Serial.print(compassValueCalc);
  //Serial.print("\tcompassValue: ");
  //Serial.print(compassValue);
  nanoCom();
  Serial.println("\tendline");/*
  char buf[50];
  sprintf(buf,"BGoalX:\t%d\tBGoalY:\t%d\tBGoalWidth:\t%d\tBGoalHeight:\t%d\n", bgoalX, bgoalY, bgoalWidth, bgoalHeight);
  Serial.print(buf);
  sprintf(buf,"YGoalX:\t%d\tYGoalY:\t%d\tYGoalWidth:\t%d\tYGoalHeight:\t%d\n", ygoalX, ygoalY, ygoalWidth, ygoalHeight);
  Serial.print(buf);
  sprintf(buf,"BallX:\t%d\tBallY:\t%d\tBallWidth:\t%d\tBallHeight:\t%d\n", ballX, ballY, ballWidth, ballHeight);
  Serial.print(buf);*/
}

void nanoCom() {
  /*if(I2CComCooldown <= millis()-10){
    I2CComCooldown = millis();*/
  /////abcdefg123Serial.print("Test");
  Wire.beginTransmission(0x30); // transmit to device #48
  //DirectionGo
  //1 = Forward
  //2 = Forward Left
  //3 = Forward Right
  //4 = Back Left
  //5 = Back Right
  //6 = Back
  //Serial.print("directionSend:\t");
  //Serial.print(directionSend);
  directionSend = Direction;

  //Serial.print("motorsOn:\t");
  //Serial.print(motorsOn);

  //Serial.print("DisableMode:\t");
  //Serial.print(DisableMode);

  Wire.write(directionSend * motorsOn * DisableMode); // sends one byte DirectionSend
  if (Straight == HIGH || motorsOn == false) {
    Identifier = 7; //7 Means don't adjust
    Wire.write(7);
  }
  else if (CorrectRight == HIGH) {
    Wire.write(8); // means correct to the right
    Identifier = 8;
  }
  else if (CorrectLeft == HIGH) {
    Wire.write(9); // correct to the left
    Identifier = 9;
  }

  // Serial.println(calibratedCompassValue);

  if (correctAmount < 60) {
    correctAmount = 60; //Setting 30 as the maximum adjustemnt.
    //correctAmount = correctAmount + 2; //Adding 2 to override motor power differences.
  }
  //Serial.print("\tcorrectAmount: ");
  //Serial.print(correctAmount);
  Wire.write(correctAmount);
  if (kicker == 27 && kickerCooldown < millis() - 5000) {
    canKick = true;
    kickerCooldown = millis();
  }
  else {
    canKick = false;
  }
  Wire.write(kicker * canKick);
  Wire.write(Identifier + correctAmount + directionSend * motorsOn * DisableMode + kicker * canKick); //sum of numbers used to help filter out bad dat
  kicker = 0;
  Wire.endTransmission();    // stop transmitting
  //}
}

void disableMotorsButton() {
  if (button[0] <= millis() - 1000 && digitalRead(22) == LOW || bypass[0] == true) {   // Button has been pressed
    if (bypass[0] == false) {
      button[0] = millis();
    }
    bypass[0] = true;
    if (button[0] < millis() - 20 && digitalRead(22) == LOW) { // Really pressed
      button[0] = millis();
      bypass[0] = false;
      if (motorsOn == true) {
        motorsOn = false;
        //abcdefg123Serial.print("\tMotorsOff");
      }
      else {
        motorsOn = true;
        //abcdefg123Serial.print("\tMotorsOn");
      }
    }
  }
}

void calibrationKickerButton() {
  if (digitalRead(24) == LOW || bypass[1] == true) {   // Button has been pressed
    if (bypass[1] == false && button[1] < millis() - 50) {
      button[1] = millis();
    }
    if (button[1] < millis() - 20 && digitalRead(24) == LOW || bypass[1] == true) { // Really pressed
      bypass[1] = true;
      if (digitalRead(24) == HIGH) { // Realsed
        if (button[1] < millis() - 2000 ) {
          kicker = 27;
          /////abcdefg123Serial.print("\tKick");
          bypass[1] = false;
        }
        else {
          calibrate();
          /////abcdefg123Serial.print("\tCalibrate");
          bypass[1] = false;

        }
      }
    }
  }
}

void jumpers() {

}

void calibrate() {
  compassCalibrate.clear();
  for (int i = 0; i < 50; i++) {
    GetHeadingDegrees();
    compassCalibrate.push(headingDegrees);
    delay(50);
  }
  /////abcdefg123Serial.print(compassCalibrate.mean());
  startCompassReading = compassCalibrate.mean();
  EEPROM.writeFloat(1, startCompassReading);
}

void GetHeadingDegrees() {
  float values_from_magnetometer[3];

  getHeading();
  values_from_magnetometer[0] = (float)xv;
  values_from_magnetometer[1] = (float)yv;
  values_from_magnetometer[2] = (float)zv;
  transformation(values_from_magnetometer);

  vector_length_stabilasation();

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  // float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  // heading += declinationAngle;


  heading = atan2(calibrated_values[0], calibrated_values[1]);


  if (heading < 0)
    heading += 2 * PI;
  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;
  // Convert radians to degrees for readability.
  if (heading_last == heading)
    headingDegrees = heading * 180 / M_PI;
  heading_last = heading;


  /////abcdefg123Serial.print("\t");
  /////abcdefg123Serial.print(calibrated_values[0]);
  /////abcdefg123Serial.print("\t");
  /////abcdefg123Serial.print(calibrated_values[1]);
  /////abcdefg123Serial.print("\t");
  /////abcdefg123Serial.print(calibrated_values[2]);
  /////abcdefg123Serial.print("\t");



}

void setupHMC5883L() {
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x01); //select configoration register B
  Wire.write(0x01); //scale 1.3
  Wire.endTransmission();
}

void getHeading() {
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();


  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if (6 <= Wire.available()) {
    xv = Wire.read() << 8; //X msb
    xv |= Wire.read(); //X lsb
    zv = Wire.read() << 8; //Z msb
    zv |= Wire.read(); //Z lsb
    yv = Wire.read() << 8; //Y msb
    yv |= Wire.read(); //Y lsb
  }
}

void compassCalibration() {
  if (compassValueCalc - startCompassReading > 180) {
    calibratedCompassValue = -1 * (360 - compassValueCalc + startCompassReading);
  }
  else if (compassValueCalc - startCompassReading < -180) {
    calibratedCompassValue = 360 - startCompassReading + compassValueCalc;
  }
  else {
    calibratedCompassValue = compassValueCalc - startCompassReading;
  }
  /* if(startCompassReading - compassValueCalc){
     calibratedCompassValue = startCompassReading - compassValueCalc;
     }
    else if(startCompassReading < compassValueCalc){
     calibratedCompassValue = startCompassReading -360 + compassValueCalc ;
    //   IF(compassValueCalc-startCompassReading>180,
    // ((360-compassValueCalc)+startCompassReading)
    //   IF(compassValueCalc-startCompassReading<-180
    //   (360-startCompassReading+compassValueCalc),compassValueCalc-startCompassReading))
    }*/
}

void Chaseball() {
  Serial.print("ChaseBall,\t");
  //1 = Forward
  //2 = Forward Left
  //3 = Forward Right
  //4 = Back Left
  //5 = Back Right
  //6 = Back
  if (ballX < 190 && ballX > 130) {
    Direction = 1;
    Serial.print("Forwards");
  }
  else if (ballX == 320 || ballX == 0) {
    Serial.print(ballX);
    Direction = 6;
    Serial.print("Backwards");
  }
  else if (ballX > 160) { //Close to left edge
    if (ballY >  60) { // makes sure the ball is not too close
      Direction = 2; // Go Forwards Left
      Serial.print("Forward Left");
    }
    else {
      Direction = 4; //Go Back Left
      Serial.print("Back Left");
    }
  }
  else if (ballX < 160) { //Close to right edge
    if (ballY > 60) {
      Direction = 3; // Go Forwards Right
      Serial.print("Forward Right");
    }
    else {
      Direction = 5; //Go Back Right
      Serial.print("Back Right");
    }
  }
  else {
    Direction = 0; //Stop
    Serial.print("Stop");
  }
  /* if (Direction == 1 and (DirectionGo == 1 or DirectionGo == 2 or DirectionGo == 3)) {
    DirectionGo = Direction;
    }
    else if (Direction == 1 and DirectionGo == 5) {
    DirectionGo = 3;
    }
    else if (Direction == 1 and DirectionGo == 4) {
    DirectionGo = 2;
    }
    else if (Direction == 1 and DirectionGo == 6) {
    DirectionGo = 4;
    }
    if (Direction == 2 and (DirectionGo == 1 or DirectionGo == 2 or DirectionGo == 4)) {
    DirectionGo = Direction;
    }
    else if (Direction == 2 and DirectionGo == 3) {
    DirectionGo = 1;
    }
    else if (Direction == 2 and DirectionGo == 6) {
    DirectionGo = 4;
    }
    else if (Direction == 2 and DirectionGo == 5) {
    DirectionGo = 6;
    }
    if (Direction == 3 and (DirectionGo == 1 or DirectionGo == 3 or DirectionGo == 5)) {
    DirectionGo = Direction;
    }
    else if (Direction == 3 and DirectionGo == 2) {
    DirectionGo = 1;
    }
    else if (Direction == 3 and DirectionGo == 6) {
    DirectionGo = 5;
    }
    else if (Direction == 3 and DirectionGo == 4) {
    DirectionGo = 2;
    }
    if (Direction == 5 and (DirectionGo == 3 or DirectionGo == 5 or DirectionGo == 6)) {
    DirectionGo = Direction;
    }
    else if (Direction == 5 and DirectionGo == 4) {
    DirectionGo = 6;
    }
    else if (Direction == 5 and DirectionGo == 1) {
    DirectionGo = 3;
    }
    else if (Direction == 5 and DirectionGo == 2) {
    DirectionGo = 1;
    }
    if (Direction == 4 and (DirectionGo == 2 or DirectionGo == 4 or DirectionGo == 6)) {
    DirectionGo = Direction;
    }
    else if (Direction == 4 and DirectionGo == 1) {
    DirectionGo = 2;
    }
    else if (Direction == 4 and DirectionGo == 5) {
    DirectionGo = 6;
    }
    else if (Direction == 4 and DirectionGo == 3) {
    DirectionGo = 5;
    }
    if (Direction == 6 and (DirectionGo == 4 or DirectionGo == 5 or DirectionGo == 6)) {
    DirectionGo = Direction;
    }
    else if (Direction == 6 and DirectionGo == 3) {
    DirectionGo = 5;
    }
    else if (Direction == 6 and DirectionGo == 2) {
    DirectionGo = 4;
    }
    else if (Direction == 6 and DirectionGo == 1) {
    DirectionGo = 3;
    }
    if (DirectionGo != LastDirectionGo) {
    t++;
    if (t == 3) {
      t = 0;
    }
    else {
      DirectionGo = LastDirectionGo;
    }
    }*/
}

void haveBallLogic() {
  Serial.print("HaveBall,\t");
  Direction = 1;
  Serial.print("Forwards,\t");
  int goalX;
  if (goalColour == blue) {
    Serial.print("Blue");
    goalX = bgoalX;
  }
  else if (goalColour == yellow) {
    Serial.print("Yellow"); 
    goalX = ygoalX;
  }
  if (Direction == 1) {
    kickerForwardsCount = 0;
    if (kickerForwardReset == true) {
      kickerForwardReset = false;
      kickerForwardTimer = millis();
    }
  }
  else {
    kickerForwardsCount++;
    if (kickerForwardsCount == 4) {
      kickerForwardTimer = millis();
      kickerForwardReset = true;
      kickerForwardsCount = 0;
    }
  }
  if (Direction == 1 && (long)kickerForwardTimer < (long)millis() - 500) {
    if (goalX >= 155 && goalX <= 165 && digitalRead(23) == LOW && motorsOn && abs(calibratedCompassValue) <= 90) {
      kickerForwardTimer = millis();
      kickerForwardReset = true;
      kicker = 27;
      nanoCom();
      Serial.print("Kicked,\t");
    }
  }
  //Kicker Code for Robot*/
}

void haveBallCalc() {
  Serial.print("\tballY,\t");
  Serial.print(ballY);
  Serial.print("\tballX,\t");
  Serial.print(ballX);
  if (ballY <= 25 && ballX < 205 && ballX > 115) { //if sensor 14 or 15 is getting a reading. pulseTime[13] > 350 ||
    haveBall = HIGH;//sets haveBall HIGH
    haveBallTime = micros();//starts haveballtimer
  }
  else if (haveBall == HIGH && haveBallTime <= micros() - 333333) { //if the robot is going in a backwards direction or has not got a reading on the ball for a second.
    haveBall = LOW;//sets haveBall LOW
  }
}

void CorrectDirection () {
  correctAmountMultiplier = 1;
  int goalX;
  if (goalColour == blue) {
    Serial.print("Blue");
    goalX = bgoalX;
  }
  else if (goalColour == yellow) {
    Serial.print("Yellow"); 
    goalX = ygoalX;
  }
  char buf[50];
  sprintf(buf, "Target Goal is %d bgoalX,\t%d\tygoalx,%d\tgoalx,\t%d\t", goalColour, bgoalX, ygoalX, goalX);
  Serial.print(buf);
  if (!(0 == goalX || 320 == goalX)) {
    Serial.print("HasBall");
    correctAmount = 100 - (abs(160 - goalX) * correctAmountMultiplier);
    if (goalX == 0 || goalX == 320) Serial.print("No Goal Pixy");
    else if (goalX <= 155 && goalX >= 165) {
      CorrectLeft = LOW;
      Straight = HIGH;
      CorrectRight = LOW;
      Serial.print("CorrectStraightPixy,\t");
    }
    else if (goalX > 165) {
      CorrectLeft = HIGH;
      Straight = LOW;
      CorrectRight = LOW;
      Serial.print("CorrectLeftPixy,\t");
    }
    else if (goalX < 155) {
      CorrectLeft = LOW;
      Straight = LOW;
      CorrectRight = HIGH;
      Serial.print("CorrectRightPixy,\t");
    }
  }
  else {
    Serial.print("Doesnot have ball");
    Serial.print(calibratedCompassValue);
    correctAmountMultiplier = 1;
    correctAmount = byte(100 - (abs(calibratedCompassValue) * correctAmountMultiplier));
    if (calibratedCompassValue > 0 && calibratedCompassValue < 0) {
      CorrectRight = LOW;
      CorrectLeft = LOW;
      Straight = HIGH;
      Serial.print("CorrectStraightCompass,\t");
    }
    else if (calibratedCompassValue < 0) {
      CorrectRight = LOW;
      Straight = LOW;
      CorrectLeft = HIGH;
      Serial.print("CorrectRightCompass,\t");
    }
    else if (calibratedCompassValue > 0) {
      CorrectLeft = LOW;
      Straight = LOW;
      CorrectRight = HIGH;
      Serial.print("CorrectLeftCompass,\t");
    }
  }
}

float Spin() { //110 FOV
  float ballAngle;
  while (calibratedCompassValue > -10) {
    pixyCam();
    directionSend = 0;
    CorrectRight = LOW;
    nanoCom();
  }
  while (calibratedCompassValue < 10) {
    pixyCam();
    directionSend = 0;
    CorrectRight = LOW;
    nanoCom();
  }
  return ballAngle;
}

void pixyCam() {
  //Serial.print("Pixy:");
  uint8_t blocks;
  blocks = pixy.getBlocks();
  lastBallX = ballX;
  lastygoalX = ygoalX;
  lastbgoalX = bgoalX;
  if (blocks) {
    for (int q = blocks - 1; q >= 0; q--) {
      pixyX = pixy.blocks[q].x;
      pixyY = pixy.blocks[q].y;
      pixyWidth = pixy.blocks[q].width;
      pixyHeight = pixy.blocks[q].height;
      pixySigniture = pixy.blocks[q].signature;
      if (pixySigniture == 1) {
        pixyTimer[0] = 0;
        ballX = pixyX;
        ballY = pixyY;
        ballWidth = pixyWidth;
        ballHeight = pixyHeight;
      }
      if (pixySigniture == 2) {
        pixyTimer[1] = 0;
        bgoalX = pixyX;
        bgoalY = pixyY;
        bgoalWidth = pixyWidth;
        bgoalHeight = pixyHeight;
      }
      if (pixySigniture == 3) {
        pixyTimer[2] = 0;
        ygoalX = pixyX;
        ygoalY = pixyY;
        ygoalWidth = pixyWidth;
        ygoalHeight = pixyHeight;
      }
    }
  }
  else if (pixyTimer[0] >= 10) {
    if (ballX > 160) {
      ballX = 320;
    }
    if (ballX < 160) {
      ballX = 0;
    }
  }
  else if (ballX == lastBallX) {
    pixyTimer[0]++;
  }
  else if (pixyTimer[1] >= 10) {
    if (bgoalX > 160) {
      bgoalX = 320;
    }
    if (bgoalX < 160) {
      bgoalX = 0;
    }
  }
  else if (bgoalX == lastbgoalX) {
    pixyTimer[1]++;
  }
  else if (pixyTimer[2] >= 10) {
    if (ygoalX > 160) {
      ygoalX = 320;
    }
    if (ygoalX < 160) {
      ygoalX = 0;
    }
  }
  else if (ygoalX == lastygoalX) {
    pixyTimer[2]++;
  }
}
