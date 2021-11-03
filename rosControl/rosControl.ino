#include <Arduino.h>

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include "driveControl.h"
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

#define DRIVE_SCALE 400

ros::NodeHandle nh;

Controller Left;
Controller Right;

Gripper Gripper;

DriveControl Control;

Servo leftServo;

void driveCallback(geometry_msgs::Twist& cmd_vel){
    bool direction = FORWARD;
    if (cmd_vel.linear.x < 0){
        direction = BACKWARD;
    }
    Left.Drive(abs(cmd_vel.linear.x)*DRIVE_SCALE, direction);
    Right.Drive(abs(cmd_vel.linear.x)*DRIVE_SCALE, direction);
}

void leftTurnCallback(geometry_msgs::Twist& cmd_vel){
    Left.Drive(abs(cmd_vel.linear.x)*DRIVE_SCALE, 0);
    Right.Drive(abs(cmd_vel.linear.x)*DRIVE_SCALE, 1);
}

void rightTurnCallback(geometry_msgs::Twist& cmd_vel){
    Left.Drive(abs(cmd_vel.linear.x)*DRIVE_SCALE, 1);
    Right.Drive(abs(cmd_vel.linear.x)*DRIVE_SCALE, 0);

}

void gripperCallback(geometry_msgs::Twist& cmd_vel){
    leftServo.write(cmd_vel.linear.x*180);
}

ros::Subscriber<geometry_msgs::Twist> driveSub("drive", driveCallback);
ros::Subscriber<geometry_msgs::Twist> leftSub("leftTurn", leftTurnCallback);
ros::Subscriber<geometry_msgs::Twist> rightSub("rightTurn", rightTurnCallback);
ros::Subscriber<geometry_msgs::Twist> gripperSub("gripper", gripperCallback);


void setup() {
    nh.initNode();
    nh.subscribe(driveSub);
    nh.subscribe(leftSub);
    nh.subscribe(rightSub);
    nh.subscribe(gripperSub);
    
    leftServo.attach(9);
    Gripper.init(LEFT_SERVO_PWM_PIN, RIGHT_SERVO_PWM_PIN, LEFT_SERVO_POS, RIGHT_SERVO_POS);
    Left.init(LEFTPWM, LEFTA, LEFTB);
    Right.init(RIGHTPWM, RIGHTA, RIGHTB);
    
    Control.setControllers(Left, Right);
}

void loop() {
    nh.spinOnce();
    delay(1);
}
