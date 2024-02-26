#include <Servo.h>
Servo ESC1, ESC2;
void setup() {
  // put your setup code here, to run once:
  ESC1.attach(2, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(3, 1000, 2000);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  ESC2.write(0);
  ESC1.write(180);
  Serial.print("Servo2:");
  Serial.print(0);
  Serial.print("  ");
  Serial.print("Servo1:");
  Serial.print(180);
  Serial.println(" ");
  delay(4000);
  for (int i = 0; i < 180; i++) {
    ESC2.write(i);
    ESC1.write(180 - i);
    Serial.println("UÇUŞA GECIYOR");
    Serial.print("Servo2:");
    Serial.print(i);
    Serial.print("  ");
    Serial.print("Servo1:");
    Serial.print(180 - i);
    Serial.println(" ");
    delay(20);
  }
  delay(4000);
  for (int i = 0; i < 180; i++) {
    Serial.println("KARADA");
    ESC2.write(180 - i);
    ESC1.write(i);
    Serial.print("Servo2:");
    Serial.print(180 - i);
    Serial.print("  ");
    Serial.print("Servo1:");
    Serial.print(i);
    Serial.println(" ");
    delay(20);
  } 
 /* ESC1.write(0);
  ESC2.write(0);
  Serial.println("0 da");
  delay(4000);
  ESC1.write(180);
  ESC2.write(180);
  Serial.println("180 de");
  delay(4000); */
}