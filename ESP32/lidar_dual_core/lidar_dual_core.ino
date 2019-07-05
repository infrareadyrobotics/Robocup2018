#include <Wire.h>
#include <VL53L0X.h>

int lidarData;

TaskHandle_t Task1;

VL53L0X lidar;

void codeForTask1( void * parameter )
{
  Wire.begin();
  lidar.init();
  lidar.setTimeout(500);
  lidar.setMeasurementTimingBudget(20000);
  lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  for (;;) {
    lidarData = lidar.readRangeSingleMillimeters();
    vTaskDelay(1);
  }
}



// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  xTaskCreatePinnedToCore(
    codeForTask1,
    "led1Task",
    1000,
    NULL,
    1,
    &Task1,
    0);


}

void loop() {
  Serial.print("lidarData:");
  Serial.println(lidarData);
}
