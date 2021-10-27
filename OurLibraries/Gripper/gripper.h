#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>
#include <Servo.h>

class Gripper{

    public:
                                
        Gripper();
        void init(int _LEFT_SERVO_PWM_PIN, int _RIGHT_SERVO_PWM_PIN, int _LEFT_SERVO_POS, int _RIGHT_SERVO_POS);
        ~Gripper();

        void Open();
        void Close();

    private:
        Servo RIGHT_SERVO;
        Servo LEFT_SERVO;
        static int RIGHT_SERVO_PWM_PIN;
        static int LEFT_SERVO_PWM_PIN;
        int RIGHT_SERVO_POS;
        int LEFT_SERVO_POS;
};

#endif