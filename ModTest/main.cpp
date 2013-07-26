
#include <iostream>
#include "MotorController.h"

#define PI 3.14159265

int main(){

	MotorController controller;
	cout << "Initial  izing galil board" << endl;
	controller.initialize();
	cout << "COntroller initialized. Sending position cmd" << endl;
	controller.positionControl(1,PI/8);
	//Sleep(10000);

}

double degToRad(double angle){

	return angle * (PI/180.0);
}
