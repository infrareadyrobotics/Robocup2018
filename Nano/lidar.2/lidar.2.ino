#include <SoftwareSerial.h>
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X lidar;
SoftwareSerial mySerial(11,10);

int lidarData;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  mySerial.begin(9600);
  
  Wire.begin();

  lidar.init();
  lidar.setTimeout(500);
  lidar.setMeasurementTimingBudget(20000);
  lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

void loop(){
  lidarData = int((float)lidar.readRangeSingleMillimeters() / 10.0);
  //lidarData = 10;
  mySerial.write(lidarData);
}


