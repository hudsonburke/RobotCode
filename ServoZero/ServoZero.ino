#include <Servo.h>

Servo leftServo;
Servo rightServo;


void setup() {
	leftServo.attach(9);
	rightServo.attach(10);
	leftServo.write(180);
	rightServo.write(0);
}


void loop() {
}
