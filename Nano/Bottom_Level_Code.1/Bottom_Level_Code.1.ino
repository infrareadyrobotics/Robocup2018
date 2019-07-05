#include <EnableInterrupt.h> //enables lots of extra interupt pins
#include <Wire.h>

#define DEBUG 1

//Colour Sensors Variables
int colourPin[7] = {
    12, 6, 7, 8, 9, 10, 11};
volatile unsigned long ColourIN[7];
unsigned long ColourTime1[7]; //Time when pin goies LOW
unsigned long ColourTime2[7]; //Time when pin goes HIGH
boolean ColourPinState[7];

int colourValue[7]; // 0=error 1=white 2=green 3=black

//Ultrasonic Variables
int frontUltraSonic;
int rightUltraSonic;
int backUltraSonic;
int leftUltraSonic;
volatile unsigned long UltraPingFront;
volatile unsigned long UltraPingRight;
volatile unsigned long UltraPingBack;
volatile unsigned long UltraPingLeft;
unsigned long UltraTime1[4]; //Time when pin goies LOW
unsigned long UltraTime2[4]; //Time when pin goes HIGH
boolean UltraEchoPinState[4];
int EchoPIN[4];
int TriggerPIN[4];
boolean FrontPing;
boolean RightPing;
boolean BackPing;
boolean LeftPing;
unsigned long PingSwitchTimer;
int PingState;

void setup()
{
  delay(500);
  Serial.begin(115200);
  // put your setup code here, to run once:
  setupColourSensors();
  setupUltrasonics();
  Wire.begin(0x30);
  Wire.onRequest(i2c);
}

void loop()
{
  detectColour();
  debugColourSensors();
  pingUltraSonics();
  calculateUltrasonics();
  debugUltrasonics();
  
  if (DEBUG)
    Serial.println();
}

void i2c()
{

  //Wire.beginTransmission(0x30); // transmit to device #48

  Wire.write(colourValue[0]);
  Wire.write(colourValue[1]);
  Wire.write(colourValue[2]);
  Wire.write(colourValue[3]);
  Wire.write(colourValue[4]);
  Wire.write(colourValue[5]);
  Wire.write(colourValue[6]);
  Wire.write(colourValue[7]);

  Wire.write(frontUltraSonic);
  Wire.write(rightUltraSonic);
  Wire.write(backUltraSonic);
  Wire.write(leftUltraSonic);

  //Wire.endTransmission(); // stop transmitting
  //}
}

///////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\
///////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\
//////////////////ColourSensors\\\\\\\\\\\\\\\\\
///////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\
///////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\

void setupColourSensors()
{
  enableInterrupt(colourPin[0], colourSensor1Read, CHANGE);
  enableInterrupt(colourPin[1], colourSensor2Read, CHANGE);
  enableInterrupt(colourPin[2], colourSensor3Read, CHANGE);
  enableInterrupt(colourPin[3], colourSensor4Read, CHANGE);
  enableInterrupt(colourPin[4], colourSensor5Read, CHANGE);
  enableInterrupt(colourPin[5], colourSensor6Read, CHANGE);
  enableInterrupt(colourPin[6], colourSensor7Read, CHANGE);
}

void debugColourSensors()
{
  if (!DEBUG)
    return;
  for (int i = 0; i < 7; i++)
  {
    Serial.print("\t Colour Sensor ");
    Serial.print(i);
    Serial.print(" is: ");
    Serial.print(ColourIN[i]);
    if (colourValue[i] == 1)
    {
      Serial.print("\tand is White");
    }
    else if (colourValue[i] == 2)
    {
      Serial.print("\tand is Green");
    }
    else if (colourValue[i] == 3)
    {
      Serial.print("\tand is Black");
    }
    else if (colourValue[i] == 0)
    {
      Serial.print("\tand is Error");
    }
  }
}

void colourSensor1Read()
{
  ColourPinState[0] = digitalRead(colourPin[0]);
  if ((ColourPinState[0] == LOW))
  {
    ColourTime1[0] = micros(); //get time of pulse going down
  }
  else
  {
    ColourTime2[0] = micros();                     //get time of pulse going up
    ColourIN[0] = ColourTime2[0] - ColourTime1[0]; //measure time between down and up
  }
}

void colourSensor2Read()
{
  ColourPinState[1] = digitalRead(colourPin[1]);
  if ((ColourPinState[1] == LOW))
  {
    ColourTime1[1] = micros(); //get time of pulse going down
  }
  else
  {
    ColourTime2[1] = micros();                     //get time of pulse going up
    ColourIN[1] = ColourTime2[1] - ColourTime1[1]; //measure time between down and up
  }
}

void colourSensor3Read()
{
  ColourPinState[2] = digitalRead(colourPin[2]);
  if ((ColourPinState[2] == LOW))
  {
    ColourTime1[2] = micros(); //get time of pulse going down
  }
  else
  {
    ColourTime2[2] = micros();                     //get time of pulse going up
    ColourIN[2] = ColourTime2[2] - ColourTime1[2]; //measure time between down and up
  }
}

void colourSensor4Read()
{
  ColourPinState[3] = digitalRead(colourPin[3]);
  if ((ColourPinState[3] == LOW))
  {
    ColourTime1[3] = micros(); //get time of pulse going down
  }
  else
  {
    ColourTime2[3] = micros();                     //get time of pulse going up
    ColourIN[3] = ColourTime2[3] - ColourTime1[3]; //measure time between down and up
  }
}

void colourSensor5Read()
{
  ColourPinState[4] = digitalRead(colourPin[4]);
  if ((ColourPinState[4] == LOW))
  {
    ColourTime1[4] = micros(); //get time of pulse going down
  }
  else
  {
    ColourTime2[4] = micros();                     //get time of pulse going up
    ColourIN[4] = ColourTime2[4] - ColourTime1[4]; //measure time between down and up
  }
}

void colourSensor6Read()
{
  ColourPinState[5] = digitalRead(colourPin[5]);
  if ((ColourPinState[5] == LOW))
  {
    ColourTime1[5] = micros(); //get time of pulse going down
  }
  else
  {
    ColourTime2[5] = micros();                     //get time of pulse going up
    ColourIN[5] = ColourTime2[5] - ColourTime1[5]; //measure time between down and up
  }
}

void colourSensor7Read()
{
  ColourPinState[6] = digitalRead(colourPin[6]);
  if ((ColourPinState[6] == LOW))
  {
    ColourTime1[6] = micros(); //get time of pulse going down
  }
  else
  {
    ColourTime2[6] = micros();                     //get time of pulse going up
    ColourIN[6] = ColourTime2[6] - ColourTime1[6]; //measure time between down and up
  }
}

void detectColour()
{
  for (int i = 0; i < 6; i++)
  {
    if (ColourIN[i] >= 0 && ColourIN[i] < 200)
    { //White
      colourValue[i] = 1;
    }
    else if (ColourIN[i] >= 500 && ColourIN[i] <= 1000)
    { //Green
      colourValue[i] = 2;
    }
    else if (ColourIN[i] >= 1001 && ColourIN[i] <= 2000)
    { //Black
      colourValue[i] = 3;
    }
    else if (ColourIN[i] >= 2001)
    { //Error
      colourValue[i] = 0;
    }
  }
}

///////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\
///////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\
//////////////////UltraSonics\\\\\\\\\\\\\\\\\\\
///////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\
///////////////////////\\\\\\\\\\\\\\\\\\\\\\\\\

void setupUltrasonics()
{
  FrontPing = LOW;
  LeftPing = LOW;
  BackPing = LOW;
  RightPing = LOW;
  EchoPIN[0] = A3;
  EchoPIN[1] = A2;
  EchoPIN[2] = A1;
  EchoPIN[3] = A0;
  TriggerPIN[0] = 5;
  TriggerPIN[1] = 4;
  TriggerPIN[2] = 3;
  TriggerPIN[3] = 2;
  pinMode(TriggerPIN[0], OUTPUT);
  pinMode(TriggerPIN[1], OUTPUT);
  pinMode(TriggerPIN[2], OUTPUT);
  pinMode(TriggerPIN[3], OUTPUT);
  pinMode(EchoPIN[0], INPUT);
  pinMode(EchoPIN[1], INPUT);
  pinMode(EchoPIN[2], INPUT);
  pinMode(EchoPIN[3], INPUT);
  PingSwitchTimer = micros();
  PingState = 1;
}

void calculateUltrasonics()
{
  frontUltraSonic = UltraPingFront / 2 / 29.39; //diveded by 2 beacuse sound has to go their and back and we only want the distance there
  rightUltraSonic = UltraPingRight / 2 / 29.39;
  backUltraSonic = UltraPingBack / 2 / 29.39; //sound travels a centimeter every 29 microseconds so we divide by 29 to get the distance in centimeters.
  leftUltraSonic = UltraPingLeft / 2 / 29.39;
}

void ReadUltrasonicFront()
{
  UltraEchoPinState[0] = digitalRead(EchoPIN[0]);
  if ((UltraEchoPinState[0] == HIGH && FrontPing == HIGH))
  {
    UltraTime1[0] = micros(); //get time of pulse going up
  }
  else
  {
    UltraTime2[0] = micros();                       //get time of pulse going down
    UltraPingFront = UltraTime2[0] - UltraTime1[0]; //measure time between up and down.
    FrontPing = LOW;
  }
}

void ReadUltrasonicRight()
{
  UltraEchoPinState[1] = digitalRead(EchoPIN[1]);
  if ((UltraEchoPinState[1] == HIGH && RightPing == HIGH))
  {
    UltraTime1[1] = micros(); //get time of pulse going up
  }
  else
  {
    UltraTime2[1] = micros();                       //get time of pulse going down
    UltraPingRight = UltraTime2[1] - UltraTime1[1]; //measure time between up and down.
    RightPing = LOW;
  }
}

void ReadUltrasonicBack()
{
  UltraEchoPinState[2] = digitalRead(EchoPIN[2]);
  if ((UltraEchoPinState[2] == HIGH && BackPing == HIGH))
  {
    UltraTime1[2] = micros(); //get time of pulse going up
  }
  else
  {
    UltraTime2[2] = micros();                      //get time of pulse going down
    UltraPingBack = UltraTime2[2] - UltraTime1[2]; //measure time between up and down.
    BackPing = LOW;
  }
}

void ReadUltrasonicLeft()
{
  UltraEchoPinState[3] = digitalRead(EchoPIN[3]);
  if ((UltraEchoPinState[3] == HIGH && LeftPing == HIGH))
  {
    UltraTime1[3] = micros(); //get time of pulse going up
  }
  else
  {
    UltraTime2[3] = micros();                      //get time of pulse going down
    UltraPingLeft = UltraTime2[3] - UltraTime1[3]; //measure time betweenup and down
    LeftPing = LOW;
  }
}

void pingUltraSonics()
{
  if (PingState == 1 && PingSwitchTimer < (micros() - 11600))
  {
    disableInterrupt(A0);
    digitalWrite(TriggerPIN[0], HIGH);
    PingSwitchTimer = micros();
    PingState = 2;
  }
  else if (PingState == 2 && PingSwitchTimer < micros() - 10)
  {
    FrontPing = HIGH;
    enableInterrupt(A3, ReadUltrasonicFront, CHANGE);
    digitalWrite(TriggerPIN[0], LOW);
    PingSwitchTimer = micros();
    PingState = 3;
  }
  else if (PingState == 3 && PingSwitchTimer < micros() - 11600)
  {
    disableInterrupt(A3);
    digitalWrite(TriggerPIN[1], HIGH);
    PingSwitchTimer = micros();
    PingState = 4;
  }
  else if (PingState == 4 && PingSwitchTimer < micros() - 10)
  {
    RightPing = HIGH;
    enableInterrupt(A2, ReadUltrasonicRight, CHANGE);
    digitalWrite(TriggerPIN[1], LOW);
    PingSwitchTimer = micros();
    PingState = 5;
  }
  else if (PingState == 5 && PingSwitchTimer < (micros() - 11600))
  {
    disableInterrupt(A2);
    digitalWrite(TriggerPIN[2], HIGH);
    PingSwitchTimer = micros();
    PingState = 6;
  }
  else if (PingState == 6 && PingSwitchTimer < micros() - 10)
  {
    BackPing = HIGH;
    enableInterrupt(A1, ReadUltrasonicBack, CHANGE);
    digitalWrite(TriggerPIN[2], LOW);
    PingSwitchTimer = micros();
    PingState = 7;
  }
  else if (PingState == 7 && PingSwitchTimer < micros() - 11600)
  {
    disableInterrupt(A1);
    digitalWrite(TriggerPIN[3], HIGH);
    PingSwitchTimer = micros();
    PingState = 8;
  }
  else if (PingState == 8 && PingSwitchTimer < micros() - 10)
  {
    LeftPing = HIGH;
    enableInterrupt(A0, ReadUltrasonicLeft, CHANGE);
    digitalWrite(TriggerPIN[3], LOW);
    PingSwitchTimer = micros();
    PingState = 1;
  }
}

void debugUltrasonics()
{
  if (!DEBUG)
    return;
  Serial.print("\tFrontUltraSonic: ");
  Serial.print(frontUltraSonic);
  Serial.print("\tRightUltrasonic: ");
  Serial.print(rightUltraSonic);
  Serial.print("\tBackUltrasonic: ");
  Serial.print(backUltraSonic);
  Serial.print("\tLeftUltrasonic: ");
  Serial.print(leftUltraSonic);
}
