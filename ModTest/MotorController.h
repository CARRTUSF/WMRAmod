
#include <exception>
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include <ctype.h>
#include "Galil.h"
using namespace std;

class MotorController {
public:
    MotorController();
   // ~MotorController();
    bool initialize(); //#debug figure out IP addr stuff for galil
    bool isInitialized(); // return initialized
    bool Stop(); //emergancy stop
    bool Stop(int motorNum); // emergancy stop a single motor
    float readPos(int motorNum); // returns the current motor angle in radians
    float readPosErr(int motorNum); // returns the error in  
    bool setMaxVelocity(int motorNum, float angularVelocity);
    bool setAccel(int motorNum, float angularAccelaration);
    bool setAccelAll(std::vector<int> acclVal);
    bool setDecel(int motorNum, float angularDecelaration);
	float encToAng(int motorNum, long enc);
    long angToEnc(int motorNum, float angle);
	bool positionControl(int motor,float angle);


private:

    bool initialized ;
    string ipAddr;
    double enc2Radian[9];
    double rad2Enc[9];
    static char motorLookup[9];
    static Galil controller;
    //bool readSettings(); // read settings 
    bool definePosition(int motorNum, float angle);
    bool setPID(int motorNum, int P, int I, int D);
    bool isValidMotor(int motorNum);
};