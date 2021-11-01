#!/bin/bash

echo "Enter the name of the sketch:"
read sketch

arduino-cli compile --fqbn arduino:avr:uno ~/RobotCode/$sketch
arduino-cli upload --port /dev/ttyACM0 --fqbn arduino:avr:uno ~/RobotCode/$sketch

