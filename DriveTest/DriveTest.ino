
#include <Arduino.h>
#include <ros.h>
#include "controller.h"

Controller Left;
Controller Right;

void setup() {
	Left.init(3, 2, 4);

}

void loop() {
  Left.Drive(255, 0);
   

}
