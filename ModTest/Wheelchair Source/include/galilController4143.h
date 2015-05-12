#ifndef GALILCONTROLLER4143_H
#define GALILCONTROLLER4143_H

#define _USE_MATH_DEFINES  // for M_PI
#include <math.h>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <exception>
#include <vector>
#include <ctype.h>
#include "matrix.h" 
#include "tinythread.h"

class client_tcpsocket;

using namespace std;
using namespace math;

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

class galilController4143 {
public:
	galilController4143();
	bool isInitialized();
	bool initialize();
	void stop();
	bool setPosition(long position,int motor);
	bool setVelocity(long velocity,int motor);
	bool setAcceleration(long acceleration,int motor);
	bool setTorque(long torque,int motor);
	long getPosition(int motor);
	long getVelocity(int motor);
	long getTorque(int motor);
	std::string command(std::string Command); // user command structure, used by MotorController
	vector<int> link_parameters;
	vector<double> enc2Radian;
	vector<double> WMRA_Theta_dot2Xphi_dot();
	Matrix DXphi_dot;

private:
	double encToAng(int motor, long encCount);
	tthread::thread* t;
	static void running(void * aArg);
	bool initialized;
	int commandGalil(char* Command, char* Response, int ResponseSize); // Galil Controller command structure, used by command()
	static client_tcpsocket sock; // The socket class used to communicate with galil controller
	bool setupSocket();
	bool startup();
	bool setDefaults(); // set defaults
	double phi_old;
	Matrix J_Dtheta2Dphi;
	vector<double> updateChairPositon();
	vector<double> wheel_axis_center_position;
	vector<double> wheel_theta_new;
	vector<double> wheel_theta_old;
	vector<double> wheel_theta_dot;
};

#endif;