#ifndef WMRA_H
#define WMRA_H

#include <iostream>
#include "tinythread.h"
#include "matrix.h" 
#include "ARM_module.h"
#include "WHEELCHAIR_module.h"
#include <vector>
#include "Windows.h"
#include <mutex>

using namespace math;

#ifndef _NO_TEMPLATE
typedef matrix<double> Matrix;
#else
typedef matrix Matrix;
#endif

namespace WMRA{
	class wmra
	{
	public:
		wmra(void);
		//~wmra(void);
		bool initialize();
		double phi;
		bool sendInputValues(); // sets input to zeros
		bool sendInputValues(vector<double> in);
		bool Jacobian_Ground2Endeffector();
		bool weighted_pseudoinverse();
		bool control_joint(double pitch, double roll);
		bool JointSpeed_limitation();
		vector<double> getInputValues();
		vector<double> Qarm;
		vector<double> link_parameters;
		vector<double> length_parameters;
		Matrix Jac;
		Matrix q_dot;
		vector<double> inputDevice;
		bool wmraDefaults();
		bool debugMode;
		int controlType;
		bool isInitialized();
		ARM_module ARM;
		WHEELCHAIR_module WHEELCHAIR;

	private:
		std::mutex mu;
		bool initialized;
		//static void running(LPVOID aArg);

	};
};


#endif;