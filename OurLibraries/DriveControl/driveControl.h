#ifndef DRIVECONTROL_H
#define DRIVECONTROL_H

#include "controller.h"

class DriveControl {

    public:
        DriveControl();
        void setControllers(Controller _Left, Controller _Right);
        ~DriveControl();
        
        void Forward(byte Speed);
        void Reverse(byte Speed);
        void TurnLeft(int Degrees);
        void TurnRight(int Degrees);

    private:
        Controller Left;
        Controller Right;

};

#endif