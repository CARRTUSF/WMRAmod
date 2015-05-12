#pragma once

#include <string>
#include <vector>

namespace WMRA{
	class Pose;
	class JointValueSet;
	enum CordFrame;
};

namespace WMRA{
	class ARM_module
	{
	public:
		ARM_module(void);
		~ARM_module(void);
		bool initialize();
		bool autonomous(WMRA::Pose dest, WMRA::CordFrame crodFr, bool blocking = true);
		bool teleoperation(WMRA::Pose deltaPose);
		bool teleoperation(WMRA::Pose deltaPose, double dt);
		bool teleoperation(WMRA::Pose dest, WMRA::CordFrame cordFr);
		bool openGripper(bool blocking = true);
		bool closeGripper(bool blocking = true);
		bool isGripperOpen();
		bool toReady(bool blocking = true);
		bool ready2Park(bool blocking = true);
		bool park2Ready(bool blocking = true);
		bool motionComplete();
		bool moveJoint(int jointNum, double angle, int ref);
		bool stop();
		WMRA::Pose getPose();
		WMRA::JointValueSet getJointAngles();
		bool isInitialized();
		bool setInitialJointAngles(WMRA::JointValueSet& joints);
		static void sendData(void* aArg);
		WMRA::JointValueSet getLastKnownJointPosition();
		std::vector<double> getPosition();
		std::string command(std::string Command);
		std::vector<double> updateArmPosition();
		std::vector<double> Q_arm_position; //radians
	};
};

