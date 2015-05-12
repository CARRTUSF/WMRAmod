
#include "ConfigReader.h"
#include "WMRA.h"
#include <time.h>
#include <iostream>

//#pragma comment(lib, "ARM_module.lib")
//#pragma comment(lib, "WHEELCHAIR_module.lib")

using namespace std;
using namespace WMRA;

wmra::wmra()
{

}

bool wmra::initialize()
{
	initialized = false;
	phi = 0;
//	mu.lock();
	inputDevice.resize(6);
//	mu.unlock();

	cout << "Initializing WMRA" << endl;

	if(!wmraDefaults())
		return 0;
	if(!ARM.initialize())
		return 0;
	if(!WHEELCHAIR.initialize())
		return 0;

	initialized = true;
	return 1;
}

bool wmra::Jacobian_Ground2Endeffector()
{ 
	controlType = 2;
	Matrix tgt(6,9);
	double L1 = length_parameters[0];
	double L2 = length_parameters[1];
	double L3 = length_parameters[2];
	double L4 = length_parameters[3];
	double L5 = length_parameters[4];
	double d1=link_parameters[0];
	double d2=link_parameters[1]; 
	double d3=link_parameters[2]; 
	double d4=link_parameters[3]; 
	double d5=link_parameters[4]; 
	double d6=link_parameters[5]; 
	double d7=link_parameters[6];
	double theta1=Qarm[0];
	double theta2=Qarm[1];
	double theta3=Qarm[2];
	double theta4=Qarm[3];
	double theta5=Qarm[4];
	double theta6=Qarm[5];
	double theta7=Qarm[6];

	//JG7 respect to armbase. so it is armbase control, using for teleoperation.

	std::cout << "HERE" << std::endl;
	
	if(controlType == 1)
	{		
	  tgt(0,0) = d4*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - d6*(sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) + d7*(cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)))) + d2*cos(theta1) + d5*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + d3*sin(theta1)*sin(theta2);
	  tgt(0,1) = -d6*(sin(theta5)*(cos(theta1)*cos(theta2)*sin(theta4) + cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta1)*cos(theta5)*sin(theta2)*sin(theta3)) - d7*(cos(theta6)*(cos(theta1)*cos(theta2)*cos(theta4) - cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4)) - sin(theta6)*(cos(theta5)*(cos(theta1)*cos(theta2)*sin(theta4) + cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)) - cos(theta1)*sin(theta2)*sin(theta3)*sin(theta5))) - d5*(cos(theta1)*cos(theta2)*cos(theta4) - cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4)) - d3*cos(theta1)*cos(theta2) - d4*cos(theta1)*sin(theta2)*sin(theta3);
	  tgt(0,2) = d5*sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) - d6*(cos(theta5)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta4)*sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d7*(sin(theta6)*(sin(theta5)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - cos(theta6)*sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d4*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3));
	  tgt(0,3) = d7*(cos(theta6)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*sin(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2))) + d5*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + d6*sin(theta5)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2));
	  tgt(0,4) = -d6*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d7*sin(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)));
	  tgt(0,5) = -d7*(sin(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) - cos(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))));
	  tgt(0,6) = 0;
	  tgt(0,7) = (L5*cos(phi)*(cos(phi) + (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2 + (L5*sin(phi)*(sin(phi) - (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2;
	  tgt(0,8) = (L5*cos(phi)*(cos(phi) - (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2 + (L5*sin(phi)*(sin(phi) + (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2;
	  
	  tgt(1,0) = 0;
	  tgt(1,1) = d7*(sin(theta6)*(cos(theta5)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)) + cos(theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta4)*sin(theta2) + cos(theta2)*cos(theta3)*sin(theta4))) - d5*(cos(theta4)*sin(theta2) + cos(theta2)*cos(theta3)*sin(theta4)) - d3*sin(theta2) - d6*(sin(theta5)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)) - cos(theta2)*cos(theta5)*sin(theta3)) + d4*cos(theta2)*sin(theta3);
	  tgt(1,2) = sin(theta2)*(d4*cos(theta3) + d6*cos(theta3)*cos(theta5) + d5*sin(theta3)*sin(theta4) - d6*cos(theta4)*sin(theta3)*sin(theta5) + d7*cos(theta6)*sin(theta3)*sin(theta4) + d7*cos(theta3)*sin(theta5)*sin(theta6) + d7*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta6));
	  tgt(1,3) = d6*sin(theta5)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)) - d7*(cos(theta6)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4))) - d5*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2));
	  tgt(1,4) = d6*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5)) + d7*sin(theta6)*(sin(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta2)*sin(theta3));
	  tgt(1,5) = -d7*(cos(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5)) + sin(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)));
	  tgt(1,6) = 0;
	  tgt(1,7) = (L5*cos(phi)*(sin(phi) - (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2 - (L5*sin(phi)*(cos(phi) + (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2;
	  tgt(1,8) = (L5*cos(phi)*(sin(phi) + (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2 - (L5*sin(phi)*(cos(phi) - (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2;
 
	  tgt(2,0) = d6*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d7*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)))) - d4*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) - d2*sin(theta1) - d5*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + d3*cos(theta1)*sin(theta2);
	  tgt(2,1) = d5*(cos(theta2)*cos(theta4)*sin(theta1) - cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4)) + d6*(sin(theta5)*(cos(theta2)*sin(theta1)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)) + cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3)) - d7*(sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta1)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)) - sin(theta1)*sin(theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta2)*cos(theta4)*sin(theta1) - cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4))) + d3*cos(theta2)*sin(theta1) + d4*sin(theta1)*sin(theta2)*sin(theta3);
	  tgt(2,2) = d5*sin(theta4)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - d6*(cos(theta5)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - d7*(sin(theta6)*(sin(theta5)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - cos(theta6)*sin(theta4)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - d4*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1));
	  tgt(2,3) = d7*(cos(theta6)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*sin(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2))) + d5*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + d6*sin(theta5)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2));
	  tgt(2,4) = - d6*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - d7*sin(theta6)*(sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)));
	  tgt(2,5) = -d7*(sin(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) - cos(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))));
	  tgt(2,6) = 0;
	  tgt(2,7) = 0;
	  tgt(2,8) = 0;

	  tgt(3,0) = 0;
	  tgt(3,1) = sin(theta1);
	  tgt(3,2) = -cos(theta1)*sin(theta2);
	  tgt(3,3) = cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3);
	  tgt(3,4) = sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2);
	  tgt(3,5) = cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) - sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4));
	  tgt(3,6) = cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))); 
	  tgt(3,7) = 0;
	  tgt(3,8) = 0;
	  
	  tgt(4,0) = 1;
	  tgt(4,1) = 0;
	  tgt(4,2) = cos(theta2);
	  tgt(4,3) = sin(theta2)*sin(theta3);
	  tgt(4,4) = cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4);
	  tgt(4,5) = sin(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta2)*sin(theta3);
	  tgt(4,6) = cos(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)) - sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5));
	  tgt(4,7) = 0;
	  tgt(4,8) = 0;

	  tgt(5,0) = 0;
	  tgt(5,1) = cos(theta1);
	  tgt(5,2) = sin(theta1)*sin(theta2);
	  tgt(5,3) = cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3);
	  tgt(5,4) = sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2);
	  tgt(5,5) = cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4));
	  tgt(5,6) = cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)));
	  tgt(5,7) = -L5/L1;
	  tgt(5,8) = L5/L1;
	}
	else if(controlType == 2)
	{
		Matrix J07(6,7);
		J07(0,0) = d4*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - d6*(sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) + d7*(cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)))) + d2*cos(theta1) + d5*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + d3*sin(theta1)*sin(theta2);
		J07(0,1) = - d6*(sin(theta5)*(cos(theta1)*cos(theta2)*sin(theta4) + cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta1)*cos(theta5)*sin(theta2)*sin(theta3)) - d7*(cos(theta6)*(cos(theta1)*cos(theta2)*cos(theta4) - cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4)) - sin(theta6)*(cos(theta5)*(cos(theta1)*cos(theta2)*sin(theta4) + cos(theta1)*cos(theta3)*cos(theta4)*sin(theta2)) - cos(theta1)*sin(theta2)*sin(theta3)*sin(theta5))) - d5*(cos(theta1)*cos(theta2)*cos(theta4) - cos(theta1)*cos(theta3)*sin(theta2)*sin(theta4)) - d3*cos(theta1)*cos(theta2) - d4*cos(theta1)*sin(theta2)*sin(theta3);
		J07(0,2) = d5*sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) - d6*(cos(theta5)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta4)*sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d7*(sin(theta6)*(sin(theta5)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta4)*cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - cos(theta6)*sin(theta4)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d4*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3));
		J07(0,3) = d7*(cos(theta6)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*sin(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2))) + d5*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + d6*sin(theta5)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2));
		J07(0,4) = - d6*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d7*sin(theta6)*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)));
		J07(0,5) = -d7*(sin(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) - cos(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))));
		J07(0,6) = 0;

		J07(1,0) = 0;
		J07(1,1) = d7*(sin(theta6)*(cos(theta5)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)) + cos(theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta4)*sin(theta2) + cos(theta2)*cos(theta3)*sin(theta4))) - d5*(cos(theta4)*sin(theta2) + cos(theta2)*cos(theta3)*sin(theta4)) - d3*sin(theta2) - d6*(sin(theta5)*(sin(theta2)*sin(theta4) - cos(theta2)*cos(theta3)*cos(theta4)) - cos(theta2)*cos(theta5)*sin(theta3)) + d4*cos(theta2)*sin(theta3);
		J07(1,2) = sin(theta2)*(d4*cos(theta3) + d6*cos(theta3)*cos(theta5) + d5*sin(theta3)*sin(theta4) - d6*cos(theta4)*sin(theta3)*sin(theta5) + d7*cos(theta6)*sin(theta3)*sin(theta4) + d7*cos(theta3)*sin(theta5)*sin(theta6) + d7*cos(theta4)*cos(theta5)*sin(theta3)*sin(theta6));
		J07(1,3) = d6*sin(theta5)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)) - d7*(cos(theta6)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4))) - d5*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2));
		J07(1,4) = d6*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5)) + d7*sin(theta6)*(sin(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta2)*sin(theta3));
		J07(1,5) = -d7*(cos(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5)) + sin(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)));
		J07(1,6) = 0;

		J07(2,0) = d6*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) - d7*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)))) - d4*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) - d2*sin(theta1) - d5*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + d3*cos(theta1)*sin(theta2);
		J07(2,1) = d5*(cos(theta2)*cos(theta4)*sin(theta1) - cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4)) + d6*(sin(theta5)*(cos(theta2)*sin(theta1)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)) + cos(theta5)*sin(theta1)*sin(theta2)*sin(theta3)) - d7*(sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta1)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta1)*sin(theta2)) - sin(theta1)*sin(theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta2)*cos(theta4)*sin(theta1) - cos(theta3)*sin(theta1)*sin(theta2)*sin(theta4))) + d3*cos(theta2)*sin(theta1) + d4*sin(theta1)*sin(theta2)*sin(theta3);
		J07(2,2) = d5*sin(theta4)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - d6*(cos(theta5)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - d7*(sin(theta6)*(sin(theta5)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - cos(theta4)*cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - cos(theta6)*sin(theta4)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - d4*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1));
		J07(2,3) = d7*(cos(theta6)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*sin(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2))) + d5*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + d6*sin(theta5)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2));
		J07(2,4) = - d6*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))) - d7*sin(theta6)*(sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)));
		J07(2,5) = -d7*(sin(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) - cos(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3))));
		J07(2,6) = 0;

		J07(3,0) = 0;
		J07(3,1) = sin(theta1);
		J07(3,2) = -cos(theta1)*sin(theta2);
		J07(3,3) = cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3);
		J07(3,4) = sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2);
		J07(3,5) = cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) - sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4));
		J07(3,6) = cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)));
		
		J07(4,0) = 1;
		J07(4,1) = 0;
		J07(4,2) = cos(theta2);
		J07(4,3) = sin(theta2)*sin(theta3);
		J07(4,4) = cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4);
		J07(4,5) = sin(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta2)*sin(theta3);
		J07(4,6) = cos(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)) - sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5));
		
		J07(5,0) = 0;
		J07(5,1) = cos(theta1);
		J07(5,2) = sin(theta1)*sin(theta2);
		J07(5,3) = cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3); 
		J07(5,4) = sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2);
		J07(5,5) = cos(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)) - sin(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4));
		J07(5,6) = cos(theta6)*(sin(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) + cos(theta4)*sin(theta1)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(cos(theta1)*sin(theta3) + cos(theta2)*cos(theta3)*sin(theta1)) - sin(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta1)*cos(theta3) - cos(theta2)*sin(theta1)*sin(theta3)));

		Matrix JwhA_3D(6,2);
		JwhA_3D(0,0) = (L5*(cos(phi) + (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2;
		JwhA_3D(0,1) = (L5*(cos(phi) - (2*L3*cos(phi) + 2*L2*sin(phi))/L1))/2;
		JwhA_3D(1,0) = (L5*(sin(phi) - (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2;
		JwhA_3D(1,1) = (L5*(sin(phi) + (2*L2*cos(phi) - 2*L3*sin(phi))/L1))/2;
		JwhA_3D(2,0) = 0;
		JwhA_3D(2,1) = 0;
		JwhA_3D(3,0) = 0;
		JwhA_3D(3,1) = 0;
		JwhA_3D(4,0) = 0;
		JwhA_3D(4,1) = 0;
		JwhA_3D(5,0) = -L5/L1;
		JwhA_3D(5,1) = L5/L1;

		Matrix Ground2Arm6x6(6,6);
		Ground2Arm6x6.Null();
		Ground2Arm6x6(0,0) = cos(phi);
		Ground2Arm6x6(0,1) = -sin(phi);
		Ground2Arm6x6(1,0) = sin(phi);
		Ground2Arm6x6(1,1) = cos(phi);
		Ground2Arm6x6(2,2) = 1;
		Ground2Arm6x6(3,3) = cos(phi);
		Ground2Arm6x6(3,4) = -sin(phi);
		Ground2Arm6x6(4,3) = sin(phi);
		Ground2Arm6x6(4,4) = cos(phi);
		Ground2Arm6x6(5,5) = 1;

		Matrix temp = Ground2Arm6x6*J07;

		Matrix JG(6,6);
		JG(0,0) = 1;
		JG(1,1) = 1;
		JG(2,2) = 1;
		JG(3,3) = 1;
		JG(4,4) = 1;
		JG(5,5) = 1;
		JG(0,5) = -sin(phi)*(d7*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)))) - d6*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) + d4*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) + d2*sin(theta1) + d5*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) - d3*cos(theta1)*sin(theta2)) - cos(phi)*(d1 + d5*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)) - d7*(sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4))) + d3*cos(theta2) + d6*(sin(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta2)*sin(theta3)) + d4*sin(theta2)*sin(theta3));
		JG(1,5) = cos(phi)*(d7*(cos(theta6)*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) + sin(theta6)*(cos(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) + sin(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)))) - d6*(sin(theta5)*(cos(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) + cos(theta1)*sin(theta2)*sin(theta4)) - cos(theta5)*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3))) + d4*(cos(theta3)*sin(theta1) + cos(theta1)*cos(theta2)*sin(theta3)) + d2*sin(theta1) + d5*(sin(theta4)*(sin(theta1)*sin(theta3) - cos(theta1)*cos(theta2)*cos(theta3)) - cos(theta1)*cos(theta4)*sin(theta2)) - d3*cos(theta1)*sin(theta2)) - sin(phi)*(d1 + d5*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4)) - d7*(sin(theta6)*(cos(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) - sin(theta2)*sin(theta3)*sin(theta5)) - cos(theta6)*(cos(theta2)*cos(theta4) - cos(theta3)*sin(theta2)*sin(theta4))) + d3*cos(theta2) + d6*(sin(theta5)*(cos(theta2)*sin(theta4) + cos(theta3)*cos(theta4)*sin(theta2)) + cos(theta5)*sin(theta2)*sin(theta3)) + d4*sin(theta2)*sin(theta3));

		Matrix temp2 = JG*JwhA_3D;
		
		for(int i=0; i<7; i++)
		{
			tgt(i,0) = temp(i,0);
			tgt(i,1) = temp(i,1);
			tgt(i,2) = temp(i,2);
			tgt(i,3) = temp(i,3);
			tgt(i,4) = temp(i,4);
			tgt(i,5) = temp(i,5);
			tgt(i,6) = temp(i,6);
			tgt(i,7) = temp2(i,0);
			tgt(i,8) = temp2(i,1);
		}
	}
	else
		tgt.Null();
	
	Jac = tgt;
	return 0;
}

bool wmra::weighted_pseudoinverse()
{
	double wheight_val = 1;
	Matrix weight(9,9);
	weight.Null();
	Matrix X_dot(6,1);

	// Input Device Values, should be updated in main program by using sendInputValues(vector<double>);
	vector<double> inDevice = getInputValues();
	X_dot(0, 0) = inDevice[0];
	X_dot(1, 0) = inDevice[1];
	X_dot(2, 0) = inDevice[2];
	X_dot(3, 0) = inDevice[3];
	X_dot(4, 0) = inDevice[4];
	X_dot(5, 0) = inDevice[5];

	for(int i=0; i<9; i++)
	{
		weight(i,i) = wheight_val;
	}
	
	Matrix weight_inv = weight.Inv();

    Matrix J_weighted_pseudoinverse = weight_inv*~Jac*!(Jac*weight_inv*~Jac); // matrix(9,6)

	q_dot = J_weighted_pseudoinverse * X_dot; // DEBUG: x_dot = omni input, matrix(6,1)
	return 1;
}

bool wmra::control_joint(double pitch, double roll)
{
	if(abs(pitch) > 0 || abs(roll)>0)
	{
        q_dot(6,0) = - roll * 0.1;
        q_dot(4,0) = pitch * 0.1;
	}

	return 1;
}


bool wmra::JointSpeed_limitation()
{

	// DEBUG: ToDo
	return 1;
}

bool wmra::sendInputValues()
{
	mu.lock();
	for(int i = 0; i<6; i++)
	{
		inputDevice[i] = 0.0;
	}
	return 1;
	mu.unlock();
}

bool wmra::sendInputValues(vector<double> in)
{
	if (in.size() == 7)
	{
		mu.lock();
		inputDevice = in;
		mu.unlock();
	}
	else
		return 0;
	return 1;
}

vector<double> wmra::getInputValues()
{
	vector<double> tgt;
	mu.lock();
	if (inputDevice.size() != 6)
	{
		std::cout << "Input device not setup correctly, defaulting to zeros" << std::endl;
		inputDevice.resize(6);
		for (int i = 0; i < 6; i++)
			inputDevice[i] = 0.0;
	}
	tgt = inputDevice;
	mu.unlock();
	return tgt;
}

bool wmra::isInitialized()
{
	return initialized;
}

bool wmra::wmraDefaults()
{
	link_parameters.resize(7);
	length_parameters.resize(5);
	Qarm.resize(7);
	controlType = 2;

	/*
	ConfigReader reader;
	reader.parseFile("WMRA_settings.conf");
	reader.setSection("RUN_MODE");
	if(reader.keyPresent("DEBUGMODE")){			
		debugMode = reader.getInt("DEBUGMODE");
	}
	else{
		cout << "'DEBUGMODE' default not found" << endl;		
		return 0;
	}*/
	return 1;
	

}
