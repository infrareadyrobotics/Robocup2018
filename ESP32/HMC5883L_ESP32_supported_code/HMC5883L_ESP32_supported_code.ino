#include <Wire.h> //I2C Arduino Library
#define address 0x1E //0011110b, I2C 7bit address of HMC5883


//Offset
float xO = 191;
float yO = 164.5;
float zO = -66;
//Raw Data x,y,x
int16_t xR, yR, zR;
//Heading variables
float headingRad, headingDeg, headingCal;
//Final Data Variables
int16_t x, y, z;
//Calibration Variables
float xRadius = 342;
float yRadius = 357;
float zRadius = 250;
float Radius = 250;


void setup() {
  //Initialize Serial and I2C communications
  Serial.begin(19200);
  Wire.begin();

  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x01); //select configoration register B
  Wire.write(0x01); //scale 1.3
  Wire.endTransmission();
}

void loop() {

  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register AKA as the data registor
  Wire.endTransmission();


  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if (6 <= Wire.available()) {
    xR = Wire.read() << 8; //X msb
    xR |= Wire.read(); //X lsb
    zR = Wire.read() << 8; //Z msb
    zR |= Wire.read(); //Z lsb
    yR = Wire.read() << 8; //Y msb
    yR |= Wire.read(); //Y lsb
  }


  x = (xR - (int)xO)*(Radius*2/xRadius);
  y = (yR - (int)yO)*(Radius*2/yRadius);
  z = (zR - (int)zO)*(Radius*2/yRadius);

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
//  Serial.print(x);
//  Serial.print("\t");
//  Serial.print(y);
//  Serial.print('\t');
  Serial.print(xR);
  Serial.print('\t');
  Serial.print(yR);
  /*Serial.print("z: ");
//  Serial.println(z);*/
//  Serial.print("Heading: ");
//  Serial.println(headingDeg, 2);
/*
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);*/


/*
  Serial.print("Xoffset ");
  Serial.println((xMax + xMin)/2);
  Serial.print("Yoffset ");
  Serial.println((yMax + yMin)/2);
  Serial.print("Zoffset ");
  Serial.println((yMax + yMin)/2);
*/
  Serial.println();
}
