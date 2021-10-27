#include "controller.h"


Controller::Controller(){
};

void Controller::init(int _PWMPIN, int _INAPIN, int _INBPIN){
    PWMPIN = _PWMPIN;
    INAPIN = _INAPIN;
    INBPIN = _INBPIN;

    pinMode(PWMPIN, OUTPUT);
    pinMode(INAPIN, OUTPUT);
    pinMode(INBPIN, OUTPUT);
    this->Stop();
}

Controller::~Controller(){};

void Controller::Drive(byte Speed, bool Forward) {
    SetPWMA(Speed);
    digitalWrite(INAPIN, Forward);
    digitalWrite(INBPIN, !Forward);
};

void Controller::Stop() {
    SetPWMA(255);
    digitalWrite(INAPIN, LOW);
    digitalWrite(INBPIN, LOW);
};

void Controller::Coast(){
    SetPWMA(255);
    digitalWrite(INAPIN, LOW);
    digitalWrite(INBPIN, LOW);
};

void Controller::SetPWMA(byte Value) {
    analogWrite(PWMPIN, Value);
};