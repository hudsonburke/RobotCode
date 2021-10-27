#include "gripper.h"


Gripper::Gripper(){
};

void Gripper::init(int _LEFT_SERVO_PWM_PIN, int _RIGHT_SERVO_PWM_PIN, int _LEFT_SERVO_POS, int _RIGHT_SERVO_POS){
    LEFT_SERVO_PWM_PIN = _LEFT_SERVO_PWM_PIN;
    RIGHT_SERVO_PWM_PIN = _RIGHT_SERVO_PWM_PIN;

    LEFT_SERVO_POS = _LEFT_SERVO_POS;
    RIGHT_SERVO_POS = _RIGHT_SERVO_POS;

    LEFT_SERVO.attach(LEFT_SERVO_PWM_PIN);
    RIGHT_SERVO.attach(RIGHT_SERVO_PWM_PIN);

    LEFT_SERVO.write(LEFT_SERVO_POS);
    RIGHT_SERVO.write(RIGHT_SERVO_POS);
}

Gripper::~Gripper(){};

void Gripper::Open() {
   RIGHT_SERVO.write(180);
   LEFT_SERVO.write(180);
   delay(100);
};

void Gripper::Close() {
   RIGHT_SERVO.write(0);
   LEFT_SERVO.write(0);
   delay(100);
};
