//#include <Arduino.h>
unsigned long lastmillis = 0;
unsigned long Printcal = millis();
unsigned long timer1 = millis();
unsigned long timer2 = millis();
unsigned long timer3 = millis();

float actualCurrent = 0.23;
float ampsInput = 5.4;
float currentConstant = 0.23 / 5.4;
float amps;
float current[10];
int PWMOn = 1;
int PWMOn2 = 0;
#define currentPin 6
#define PWMPin 27
#define In1Pin 25
#define In2Pin 26
int PWMPower = 100;




void setup() {
  Serial.begin(115200);
  Serial.println("TEST");

  pinMode(currentPin, INPUT);
  Serial.println("TEST");


  //pinMode(PWMPin, OUTPUT);
    Serial.println("TEST");

  //pinMode(In1Pin, OUTPUT);
    Serial.println("TEST");

  //pinMode(In2Pin, OUTPUT);
    Serial.println("TEST");

  ledcSetup(PWMPin, 12000, 8);
    Serial.println("TEST");

}

void loop() {
  Serial.println("TEST");
  /*
    if (amps >= 1.50) { // amps before pwm fall off
    if ( timer3 < (millis() - 1000 )) {//Time before pwm fall off
      if (PWMPower > 20 && timer2 < (millis() - 50)) {// min pwm percent
        timer2 = millis();
        PWMPower -= 5; // Pwm decrement per time interval
      }
    }
    }
    else if (PWMPower != 100) { // if pwm is not at 100%
    if (PWMPower < 100 && timer2 < (millis() - 50)) { // increment until pwm is at or above 100% each time interval
      timer2 = millis();
      PWMPower += 10; // Pwm increment per time interval
    }
    if (PWMPower > 100) {
      PWMPower = 100; //set pwm to 100% if is above 100%
    }
    }
    else {
    timer3 = millis(); //reset timer if pwm is at 100% and the current is less than limited amount.
    }*/
  if ((long)timer3 < (long)millis() - 10000) {
    timer3 = millis();
    PWMPower -= 10;
    if (PWMPower == 20) {
      PWMPower = 100;
    }
  }
  if ((long)timer2 < (long)millis() - 5000 && PWMOn == HIGH) {
    timer2 = millis();
    PWMOn = LOW;
    PWMOn2 = HIGH;
  }
  if ((long)timer2 < (long)millis() - 5000 && PWMOn == LOW) {
    timer2 = millis();
    PWMOn = HIGH;
    PWMOn2 = LOW;
  }
  ledcWrite(PWMPin, PWMPower * 2.55);

  //analogWrite(PWMPin, PWMPower * 2.55);
  digitalWrite(In1Pin, PWMOn);
  digitalWrite(In2Pin, PWMOn2);


  // clear the internal memory

  //==============================================================================
  //     Calculate and display RPMs of lap.
  //===============================================================================
  /*if (millis() - lastmillis >= 15000){  //
     disableInterrupt(3);    //Disable interrupt when calculating
     rpm = rpmcount * 4;  // Convert frecuency to RPM. Four Magnets, each at 90 degrees.
     rpmcount = 0; // Restart the RPM counter
     lastmillis = millis(); // Uptade lasmillis
     enableInterrupt(3, rpmcounter, RISING); //enable interrupt
      }*/

  // print volts

  //print current
  //avgCurrent();
  Serial.print(PWMPower);
  Serial.print("\t");
  Serial.print(amps, 2);
  Serial.print("\t");
  Serial.print(PWMOn);
  Serial.print("\t");
  // END of loop()
}

void avgCurrent() {
  char output[50];
  sprintf(output, "Size of current: %d", sizeof(current));
  Serial.println(output);
  for (int i = sizeof(current) / 4 - 1; i > 0 / 4; i--) {
    current[i] = current[i - 1];
    sprintf(output, "Val %d is %d", i + 1, (int)current[i]);
    Serial.println(output);
  }
  current[0] = 1;//analogRead(currentPin);
  sprintf(output, "Val %d is %d", 1, (int)current[0]);
  Serial.println(output);
  for (int i = 0; i < sizeof(current) / 4; i++) {
    amps += current[i];
  }
  amps /= sizeof(current) / 4;
  sprintf(output, "Amps: %d", (int)amps);
  Serial.println(output);
}
