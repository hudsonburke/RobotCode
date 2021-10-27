#include "driveControl.h"

DriveControl::DriveControl(){};

void DriveControl::setControllers(Controller _Left, Controller _Right){
    Left = _Left;
    Right = _Right;

    Left.Stop();
    Right.Stop();
};

DriveControl::~DriveControl(){};

void DriveControl::Forward(byte Speed){
    Left.Drive(Speed, 1);
    Right.Drive(Speed, 1);
}

void DriveControl::Reverse(byte Speed){
    Left.Drive(Speed, 0);
    Right.Drive(Speed, 0);
}

void DriveControl::TurnLeft(int Degrees){
    Left.Drive(255, 0);
    Right.Drive(255, 1);
    delay(Degrees); // need to do some conversion here
    Left.Stop();
    Right.Stop();
}

void DriveControl::TurnRight(int Degrees){
    Left.Drive(255, 1);
    Right.Drive(255, 0);
    delay(Degrees); // need to do some conversion here
    Left.Stop();
    Right.Stop();
}


