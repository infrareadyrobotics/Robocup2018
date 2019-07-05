#include <Coordinates.h>



Coordinates point = Coordinates();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(33);
  Serial.println("Power On");
}

void loop() {
  if (Serial.available()) {
    Decode(Serial.readString());
  }
}

int Decode(String remaning) {
  //String original = "[39 x 34 from (338, 446)]";
  String x;
  String y;
  String sizex;
  String sizey;
  int intx;
  int inty;
  int intsizex;
  int intsizey;
  float getR();

  //remaning = original;
  Serial.print(remaning);
  sizex = remaning.substring(remaning.indexOf('\x1e') + 1, remaning.indexOf('\x1f'));

  remaning = remaning.substring(remaning.indexOf('\x1f') + 1);
  sizey = remaning.substring(0, remaning.indexOf('\x1f'));

  remaning = remaning.substring(remaning.indexOf('\x1f') + 1);
  x = remaning.substring(0, remaning.indexOf('\x1f'));

  remaning = remaning.substring(remaning.indexOf('\x1f') + 1);
  y = remaning.substring(0, remaning.indexOf('\x1d'));

  intsizex = sizex.toInt();
  intsizey = sizey.toInt();
  intx = x.toInt() + intsizex / 2;
  inty = y.toInt() + intsizey / 2;

  point.fromCartesian(intx, inty);

  Serial.print("R: ");
  Serial.print(point.getAngle());
  Serial.print("sizex: ");
  Serial.print(sizex);
  Serial.print(" sizey: ");
  Serial.print(sizey);
  Serial.print(" x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.println(y);
  return (point.getAngle());
}

