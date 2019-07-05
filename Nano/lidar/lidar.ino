
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X lidar;

int lidarData;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
// pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin(0x05);
  Wire.onRequest(i2c);
  
  //Wire.begin();
//
//  lidar.init();
//  lidar.setTimeout(500);
//  lidar.setMeasurementTimingBudget(20000);
//  lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
//  lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

void loop()
{
//  Serial.println(lidarData);
  lidarData = int((float)lidar.readRangeSingleMillimeters() / 10.0);
}


void i2c()
{
//  digitalWrite(LED_BUILTIN, HIGH); 
//  Serial.print("I2C");
//  lidarData = 10;
  Wire.write(10);
}

