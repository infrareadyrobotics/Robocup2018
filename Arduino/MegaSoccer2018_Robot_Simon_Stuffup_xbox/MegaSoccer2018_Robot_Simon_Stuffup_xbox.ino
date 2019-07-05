#include <Coordinates.h>

//#include <EEPROMex.h>
//#include <EEPROMVar.h>
//#include <Average.h>
#include <Wire.h>
#include <SoftwareSerial.h>

#define address 0x1E //0011110b, I2C 7bit address of HMC5883

Coordinates lStick = Coordinates();
float rotAmt;
float lx;
float ly;
float rx;
int intrx;
float ry;
int kick;
int autoMove;
int rotNo;
int directionSend;
uint8_t Direction;
boolean motorsOn = true;
byte kicker = 0;
boolean DisableMode = true;
byte Identifier;
boolean Straight = HIGH;
boolean CorrectRight;
boolean CorrectLeft;
byte correctAmount;
unsigned long kickerCooldown;
boolean canKick;
long dataTime;
SoftwareSerial mySerial(7, 11);

void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
  Serial1.begin(115200);
  Wire.begin();
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

}

void loop() {
  String blueData;
//  nanoCom();
//  delay(50);
  if (millis() - dataTime > 500){
    Direction = 6;
    directionSend = 7;
    intrx = 0;
    nanoCom();
  }
  if (Serial1.available()) {
    dataTime = millis();
    kick = 0;
    blueData = Serial1.readStringUntil('_');
    Serial.println("data: ");
    Serial.println("-------------");
    Serial.print(blueData);
    Serial.println("-------------");
    blueData = blueData.substring(blueData.indexOf('+') + 1);

    if (blueData.substring(0, 1) == "l") {
      Serial.print("Translate: ");
      lx = blueData.substring(1, blueData.indexOf(',')).toFloat();
      ly = blueData.substring(blueData.indexOf(',') + 1).toFloat();
    } else if (blueData.substring(0, 1) == "r") {
      Serial.print("Rotate: ");
      rx = blueData.substring(1, blueData.indexOf(',')).toFloat();
      ry = blueData.substring(blueData.indexOf(',') + 1).toFloat();
    } else if (blueData.substring(0, 1) == "k") {
      Serial.print("Kick: ");
      kick = blueData.substring(blueData.indexOf(',') + 1).toFloat();
    } else if (blueData.substring(0, 1) == "a") {
      Serial.print("Auto: ");
      autoMove = blueData.substring(blueData.indexOf(',') + 1).toFloat();
    } else {
      Serial.println("ERROR");
    }
    lStick.fromCartesian(lx * 10, ly * 10);
    rotAmt = lStick.getAngle();
    rotAmt = rotAmt * 57.2957795131;
    //    rotAmt = rotAmt;
    rotNo = round(rotAmt);
//    Serial.print("lx: ");
//    Serial.print(lx);
//    Serial.print("ly: ");
//    Serial.print(ly);
//    Serial.print("rx: ");
//    Serial.print(rx);
//    Serial.print("ry: ");
//    Serial.print(ry);
//    Serial.print("kick: ");
//    Serial.print(kick);
//    Serial.print("auto: ");
//    Serial.print(autoMove);
//    Serial.print("rotAmt: ");
//    Serial.print(rotAmt);
//    Serial.print("rotNo: ");
//    Serial.print(rotNo);
    Serial.print("lStick.getR: ");
    Serial.print(float(lStick.getR()));
    if (lStick.getR() < 400) {
      Serial.println("stop");
      Direction = 0;
      motorsOn = 0;
    } else if (rotNo <= 300 && rotNo >= 240) {
      Serial.println("forward");
      Direction = 1;//forward
      motorsOn = 1;
    } else if (rotNo <= 240 && rotNo >= 180) {
      Serial.println("Forward Left");
      Direction = 2;//Forward Left
      motorsOn = 1;
    } else if (rotNo <= 360 && rotNo >= 300) {
      Serial.println("Forward Right");
      Direction = 3;//Forward Right
      motorsOn = 1;
    } else if (rotNo <= 180 && rotNo >= 120) {
      Serial.println("Back Left");
      Direction = 4;//Back Left
      motorsOn = 1;
    } else if (rotNo <= 60 && rotNo >= 0) {
      Serial.println("Back Right");
      Direction = 5;//Back Right
      motorsOn = 1;
    } else if (rotNo <= 120 && rotNo >= 60) {
      Serial.println("Back");
      Direction = 6;//Back
      motorsOn = 1;
    }
    kicker = kick;
    nanoCom();
  }
}
void nanoCom() {
  Wire.beginTransmission(0x30); // transmit to device #48
  //DirectionGo
  //1 = Forward
  //2 = Forward Left
  //3 = Forward Right
  //4 = Back Left
  //5 = Back Right
  //6 = Back
//  Direction = 1;
  directionSend = Direction;

//  Wire.write(directionSend * motorsOn * DisableMode); // sends one byte DirectionSend
  Wire.write(directionSend); // sends one byte DirectionSend
  if (rx == 0) {
    Identifier = 7; //7 Means don't adjust
    Wire.write(7);
  }
  else if (rx > 0) {
    Wire.write(8); // means correct to the right
    Identifier = 8;
  }
  else if (rx < 0) {
    Wire.write(9); // correct to the left
    Identifier = 9;
  }

  // Serial.println(calibratedCompassValue);
  intrx = int(rx);
  intrx = abs(intrx);
  if (intrx > 60) {
    intrx = 60; //Setting 30 as the maximum adjustemnt.
    //correctAmount = correctAmount + 2; //Adding 2 to override motor power differences.
  }
  if (intrx < 15) {
    intrx = 0; //Setting 30 as the maximum adjustemnt.
    //correctAmount = correctAmount + 2; //Adding 2 to override motor power differences.
  }
  //Serial.print("\tcorrectAmount: ");
  //Serial.print(correctAmount);
  Wire.write(intrx);
Serial.println("int(abs(rx))");
  Serial.println(intrx);
  Serial.println("int(abs(rx))");
  if (kicker == 100 && kickerCooldown < millis() - 5000) {
    canKick = true;
    kickerCooldown = millis();
  }
  else {
    canKick = false;
  }
  Wire.write(kicker * canKick);
  Wire.write(int(lStick.getR()/6));
  Wire.endTransmission();
}
