#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

#define FORWARD 1
#define BACKWARD 0

class Controller{

    public:
                                
        Controller();
        void init(int _PWMPIN, int _INAPIN, int _INBPIN);
        ~Controller();

        void Drive(byte Speed, bool Forward);
        void Stop();
        void Coast();

    private:
        int PWMPIN;
        int INAPIN;
        int INBPIN;
        void SetPWMA(byte Value);

};

#endif