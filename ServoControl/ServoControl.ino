#include <Arduino.h>
#include <Servo.h>

Servo leftServo;
Servo rightServo; 
void setup() {
	leftServo.attach(9);
	rightServo.attach(10);
	leftServo.write(0);
	rightServo.write(180);
}

void loop() {
}
