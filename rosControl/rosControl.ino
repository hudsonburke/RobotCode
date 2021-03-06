#include <Arduino.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "controller.h"
#include "gripper.h"

#define LEFTPWM 3
#define LEFTA 2
#define LEFTB 4

#define RIGHTPWM 6
#define RIGHTA 5
#define RIGHTB 7

#define LEFT_SERVO_PWM_PIN 9
#define RIGHT_SERVO_PWM_PIN 10
#define LEFT_SERVO_POS 0
#define RIGHT_SERVO_POS 180

#define DRIVE_SCALE 255
#define TURN_SCALE 127

ros::NodeHandle nh;

Controller Left;
Controller Right;

Gripper Gripper;

Servo leftServo;

void driveCallback(geometry_msgs::Twist& cmd_vel){

    float drive_speed = cmd_vel.linear.x;
    float turn_speed = cmd_vel.angular.z;

    float left_speed = drive_speed*DRIVE_SCALE - turn_speed*TURN_SCALE;
    float right_speed = drive_speed*DRIVE_SCALE + turn_speed*TURN_SCALE;
    
    Left.Drive(left_speed);
    Right.Drive(right_speed);
}


void gripperCallback(geometry_msgs::Twist& cmd_vel){
    leftServo.write(cmd_vel.linear.x*180);
}

ros::Subscriber<geometry_msgs::Twist> driveSub("cmd_vel", driveCallback);
ros::Subscriber<geometry_msgs::Twist> gripperSub("gripper", gripperCallback);


void setup() {
    nh.initNode();
    nh.subscribe(driveSub);
    nh.subscribe(gripperSub);
    
    leftServo.attach(9);
    Gripper.init(LEFT_SERVO_PWM_PIN, RIGHT_SERVO_PWM_PIN, LEFT_SERVO_POS, RIGHT_SERVO_POS);
    Left.init(LEFTPWM, LEFTA, LEFTB);
    Right.init(RIGHTPWM, RIGHTA, RIGHTB);
}

void loop() {
    nh.spinOnce();
    delay(1);
}
