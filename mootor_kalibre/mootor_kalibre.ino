#include <Servo.h>

Servo ESC1,ESC2,ESC3,ESC4;     // create servo object to control the ESC

int potValue;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 9
  ESC1.attach(3,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(5,1000,2000);
  ESC3.attach(9,1000,2000);
  ESC4.attach(10,1000,2000); 
  Serial.begin(9600);
  ESC1.write(180);
  ESC2.write(180);
  ESC3.write(180);
  ESC4.write(180);
  delay(2000);
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0);
  delay(2000);
//  for(int i=180; i<70; i--) {
//    ESC.write(i);// Send the signal to the ESC
//    delay(100);
//  }
}

void loop() {
//  potValue = analogRead(A1);   // reads the value of the potentiometer (value between 0 and 1023)
//  potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)

  Serial.println(potValue);
  
//  ESC.write(potValue);// Send the signal to the ESC
  ESC1.write(30);
  ESC2.write(30);
  ESC3.write(30);
  ESC4.write(30);
  delay(4000);
  ESC1.write(60);
  ESC2.write(60);
  ESC3.write(60);
  ESC4.write(60);
  delay(4000);
  ESC1.write(90);
  ESC2.write(90);
  ESC3.write(90);
  ESC4.write(90);
  delay(4000);
  
}
