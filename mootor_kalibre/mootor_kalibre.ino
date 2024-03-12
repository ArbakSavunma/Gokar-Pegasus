#include <Servo.h>

Servo ESC1,ESC2,ESC3,ESC4,ESC5,ESC6,ESC7,ESC8;     // create servo object to control the ESC

int potValue;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 9
  ESC1.attach(2,1000,2000); // (pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(3,1000,2000);
  ESC3.attach(4,1000,2000);
  ESC4.attach(5,1000,2000); 
  ESC5.attach(10,1000,2000); 
  ESC6.attach(1,1000,2000);
  ESC7.attach(0,1000,2000);
  ESC8.attach(51,1000,2000); 
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
  ESC1.write(85);
  ESC2.write(85);
  ESC3.write(85);
  ESC4.write(85);
  delay(2000); 
//  for(int i=180; i<70; i--) {
//    ESC.write(i);// Send the signal to the ESC
//    delay(100);
//  }
}

void loop() {
  potValue = analogRead(A1);   // reads the value of the potentiometer (value between 0 and 1023)
  potValue = map(potValue, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)

  Serial.println(potValue);
  
  ESC1.write(potValue);// Send the signal to the ESC
  ESC2.write(potValue);
  ESC3.write(potValue);
  ESC4.write(potValue);
  ESC5.write(potValue);
  ESC6.write(potValue);
  ESC7.write(potValue);
  ESC8.write(potValue);
/*
  ESC1.write(90);
  ESC2.write(90);
  ESC3.write(90);
  ESC4.write(90);
  delay(4000);
  ESC1.write(100);
  ESC2.write(100);
  ESC3.write(100);
  ESC4.write(100);
  delay(4000);
  ESC1.write(90);
  ESC2.write(90);
  ESC3.write(90);
  ESC4.write(90);
  delay(4000);
  ESC1.write(75);
  ESC2.write(75);
  ESC3.write(75);
  ESC4.write(75);
  delay(4000);
  */
}
