
#define _USE_MATH_DEFINES

#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include "stringUtility.h"
#include "MotorController.h"

using namespace std;

// PI = M_PI;

#define PI 3.14159265



char MotorController::motorLookup[] = {'H','A','B','C','D','E','F','G','H'};
Galil MotorController::controller("192.168.1.104");

MotorController::MotorController()
{
	initialized = false;
	//controller = Galil("192.168.1.104");
	//motorLookup[] = {'H','A','B','C','D','E','F','G','H'};

	//calculate conversion values
	enc2Radian[1] = PI/6595000;//a
	enc2Radian[2]= PI/5100000;//b
	enc2Radian[3]= PI/1800000;//c
	enc2Radian[4]= PI/2300000;//d
	enc2Radian[5]= PI/2000000;//e
	enc2Radian[6]= -PI/2000000;//f
	enc2Radian[7]= PI/1500000; //g

	enc2Radian[1] = 6595000/PI;//a
	enc2Radian[2]= 5100000/PI;//b
	enc2Radian[3]= 1800000/PI;//c
	enc2Radian[4]= 2300000/PI;//d
	enc2Radian[5]= 2000000/PI;//e
	enc2Radian[6]= 2000000/PI;//f
	enc2Radian[7]= 1500000/PI; //g

	//read settings
	//call initialize();
	//set initilez flag
}

bool MotorController::initialize(){
	// set PID values

	// set motor types as in brushed motr etc..

	controller.command("BRA=1");
	controller.command("BRB=1");
	controller.command("BRC=1");
	controller.command("BRD=1");

	//set all motors to position tracking mode
	controller.command("PTA=1");
	controller.command("PTB=1");
	controller.command("PTC=1");
	controller.command("PTD=1");
	controller.command("PTE=1");
	controller.command("PTF=1");
	controller.command("PTG=1");
	controller.command("PTH=1");

	// set accelaration decelaration values
	//#debug CHANGE LATER TO PROPER ANGLES


	setDecel(1, encToAng(1,90000));
	setDecel(2, encToAng(2,90000));
	setDecel(3, encToAng(3,90000));
	setDecel(4, encToAng(4,90000));
	setDecel(5, encToAng(5,90000));
	setDecel(6, encToAng(6,90000));
	setDecel(7, encToAng(7,90000));

	//set accelaration smoothing 
	controller.command("IT*=0.6");	

	// ready position, #debug I believe this should be a function of its own, and the ready position should be recovered from the text file as well.
	definePosition(1, (PI/2));
	definePosition(2, (PI/2));
	definePosition(3, 0);
	definePosition(4, (PI/2));
	definePosition(5, (PI/2));
	definePosition(6, (PI/2));
	definePosition(7, 0);
	definePosition(8, 0);

	initialized = true;
	return true;
}

bool MotorController::isInitialized() // return initialized
{
	return initialized;
}

bool MotorController::Stop() //emergancy stop
{
	controller.command("ST ABCDEFGH");
	return true; // #debug does this return need to happen after the Arm has fully stopped?
}

bool MotorController::Stop(int motorNum) // emergancy stop a single motor
{

	if(isValidMotor(motorNum)){
		char motor = motorLookup[motorNum];
		controller.command("ST " + motor);
		return true;
	}
	else{
		return false;
	}

}
bool MotorController::setPID(int motorNum, int P, int I, int D){
	return false;
}


float MotorController::readPos(int motorNum) // returns the current motor angle in radians
{
	long encoderVal;	
	string result;
	char motor;
	if ( isValidMotor(motorNum)){
		motor = motorLookup[motorNum];
		result = controller.command( "TP" + motor);	
		istringstream stream(result);
		stream >> encoderVal;
		return encToAng(motorNum, encoderVal);        
	}
	else{
		cerr << "The motor specified is not valid" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}
}

float MotorController::readPosErr(int motorNum) // returns the error in  
{

	long encoderVal;	
	string result;
	char motor;
	if ( isValidMotor(motorNum)){
		motor = motorLookup[motorNum];
		result = controller.command( "TE" + motor);	
		istringstream stream(result);
		stream >> encoderVal;
		return encToAng(motorNum, encoderVal);        
	}
	else{
		cerr << "The motor specified is not valid" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}
}

bool MotorController::setMaxVelocity(int motorNum, float angularVelocity)
{
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angularVelocity));
		char motor = motorLookup[motorNum];
		if ((encVal >= 0) && (encVal < 12000000)){
			string command = "SP" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The velocity is outside the range" << endl;
			throw std::out_of_range ("velocity out_of_range");            
		}
	}
	else{
		cerr << "The motor specified is not valid" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}

}

bool MotorController::setAccel(int motorNum, float angularAccelaration)
{	
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angularAccelaration));
		char motor = motorLookup[motorNum];
		if ((encVal >= 1024) && (encVal <= 67107840)){
			string command = "AC" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else {
			cerr << "The Accelaration is outside the range" << endl;
			throw std::out_of_range ("Accelaration out_of_range");            
		}
	}
	else {
		cerr << "The motor specified is not valid" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}

}

bool MotorController::setDecel(int motorNum, float angularDecelaration)
{
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angularDecelaration));
		char motor = motorLookup[motorNum];
		if ((encVal >= 1024) && (encVal <= 67107840)){
			string command = "DC" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The Decelaration is outside the range" << endl;
			throw std::out_of_range ("Decelaration out_of_range");            
		}
	}
	else{
		cerr << "The motor specified is not valid" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}
}

bool MotorController::definePosition(int motorNum,float angle)
{
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angle));
		char motor = motorLookup[motorNum];
		if ((encVal >= -2147483647) && (encVal <= 2147483648)){
			string command = "DP" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The Position is outside the range" << endl;
			throw std::out_of_range ("Position out_of_range");            
		}
	}
	else{
		cerr << "The motor specified is not valid" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}
}

bool MotorController::positionControl(int motorNum,float angle)
{
	if(isValidMotor(motorNum)){
		long encVal = abs(angToEnc(motorNum,angle));
		char motor = motorLookup[motorNum];
		if ((encVal >= -2147483647) && (encVal <= 2147483648)){
			string command = "PA" + motor;
			controller.command(command + "=" + toString(encVal));
			return true;            
		}
		else{
			cerr << "The Position is outside the range" << endl;
			throw std::out_of_range ("Position out_of_range");            
		}
	}
	else{
		cerr << "The motor specified is not valid" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}
}

/*------------------------------------------------------

Private Functions

------------------------------------------------------*/

inline bool MotorController::isValidMotor(int motorNum){
	if( motorNum >= 1 || motorNum <= 8) return true;
	else return false;
}

bool readSettings() // Returns 0 is no file found or error occured
{
	return true;
}

float MotorController::encToAng(int motorNum, long encCount) // #debug needs to be finished, Also need to check initialized
{
	if (motorNum < 9 && motorNum > 0){
		return encCount * enc2Radian[motorNum]; 
	}
	else{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}
}

long MotorController::angToEnc(int motorNum, float encCount) // #debug needs to be finished, Also need to check initialized
{
	if (motorNum < 9 && motorNum > 0){
		return encCount * rad2Enc[motorNum]; 
	}
	else{
		cerr << "motor number outside range" << endl;
		throw std::out_of_range ("MotorNum out_of_range");
	}
}



