#include <SoftwareSerial.h>
#include <SPI.h>
#include <Pixy.h>
#include <Wire.h>

// This is the main Pixy object
Pixy pixy;
SoftwareSerial mySerial(9, 8);

int ballX;
int ballY;
int ballHeight;
int ballWidth;

int bgoalX;
int bgoalY;
int bgoalHeight;
int bgoalWidth;

int ygoalX;
int ygoalY;
int ygoalHeight;
int ygoalWidth;

int pixyTimer[3];

int lastBallX;
int lastygoalX;
int lastbgoalX;

int lidarData;


void setup() {
  Serial.begin(115200);
  Serial.print("Starting...\n");
  mySerial.begin(9600);

  pixy.init();

  Wire.begin(0x01);
  Wire.onRequest(i2c);
}

void loop()
{
  uint16_t blocks;
  
  lastBallX = ballX;
  lastygoalX = ygoalX;
  lastbgoalX = bgoalX;

  // grab blocks!
  blocks = pixy.getBlocks();

  // If there are detect blocks, print them!
  if (blocks) {
    for (int j = blocks - 1; j >= 0; j--) {
      if (pixy.blocks[j].signature == 1)
      {
        ballX = pixy.blocks[j].x;
        ballY = pixy.blocks[j].y;
        ballHeight = pixy.blocks[j].height;
        ballWidth = pixy.blocks[j].width;
        pixyTimer[0] = 0;
      }
      if (pixy.blocks[j].signature == 2)
      {
        bgoalX = pixy.blocks[j].x;
        bgoalY = pixy.blocks[j].y;
        bgoalHeight = pixy.blocks[j].height;
        bgoalWidth = pixy.blocks[j].width;
        pixyTimer[1] = 0;
      }
      if (pixy.blocks[j].signature == 3)
      {
        ygoalX = pixy.blocks[j].x;
        ygoalY = pixy.blocks[j].y;
        ygoalHeight = pixy.blocks[j].height;
        ygoalWidth = pixy.blocks[j].width;
        pixyTimer[2] = 0;
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

  Serial.print("lidarData: ");
  Serial.print(lidarData);
  if (mySerial.available()) {
    lidarData = mySerial.read();
    Serial.print("lidarData: ");
    Serial.print(lidarData);
  }
}

void intergerWrite(int num2) {
  byte num[2];
  num[0] = (num2 >> 8) & 0xFF;
  num[1] = num2 & 0xFF;
  Serial.print(num[0], BIN);
  Serial.print('\t');
  Serial.print(num[1], BIN);
  Serial.println();
  Wire.write(num[0]);
  Wire.write(num[1]);
}

void i2c()
{
  Serial.print("I2C: \n");

  intergerWrite(ballX);
  intergerWrite(ballY);
  intergerWrite(ballWidth);
  intergerWrite(ballHeight);

  intergerWrite(ygoalX);
  intergerWrite(ygoalY);
  intergerWrite(ygoalHeight);
  intergerWrite(ygoalWidth);

  intergerWrite(bgoalX);
  intergerWrite(bgoalY);
  intergerWrite(bgoalHeight);
  intergerWrite(bgoalWidth);

  intergerWrite(lidarData);
}
