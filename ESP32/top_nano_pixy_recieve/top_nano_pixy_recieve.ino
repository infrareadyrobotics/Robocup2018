void i2c() {

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
  }
}
