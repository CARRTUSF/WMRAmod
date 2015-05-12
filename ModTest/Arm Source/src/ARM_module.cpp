#include "ARM_module.h"
#include "Arm.h"
#include "WmraTypes.h"
#include "tinythread.h"

using namespace WMRA;
Arm arm;

ARM_module::ARM_module(void)
{
	//arm.initialize();
}

bool ARM_module::initialize()
{
	if(this->isInitialized() == true)
		return 1;
	else
		return arm.initialize();
}

bool ARM_module::autonomous(WMRA::Pose dest, WMRA::CordFrame crodFr, bool blocking)
{
	return arm.autonomous(dest, crodFr, blocking);
}

bool ARM_module::teleoperation(WMRA::Pose deltaPose)
{
	return arm.teleoperation(deltaPose);
}

bool ARM_module::teleoperation(WMRA::Pose deltaPose, double dt)
{
	return arm.teleoperation(deltaPose,dt);
}

bool ARM_module::teleoperation(WMRA::Pose dest, WMRA::CordFrame cordFr)
{
	return arm.teleoperation(dest, cordFr);
}

bool ARM_module::openGripper(bool blocking)
{
	return arm.openGripper(blocking);
}

bool ARM_module::closeGripper(bool blocking)
{
	return arm.closeGripper(blocking);
}

bool ARM_module::isGripperOpen()
{
	return arm.isGripperOpen();
}

bool ARM_module::toReady(bool blocking)
{
	return arm.toReady(blocking);
}

bool ARM_module::ready2Park(bool blocking)
{
	return arm.ready2Park(blocking);
}

bool ARM_module::park2Ready(bool blocking)
{
	return arm.park2Ready(blocking);
}

bool ARM_module::motionComplete()
{
	return arm.motionComplete();
}

bool ARM_module::moveJoint(int jointNum, double angle, int ref)
{
	return arm.moveJoint(jointNum, angle, ref);
}

bool ARM_module::stop()
{
	return arm.stop();
}

WMRA::Pose ARM_module::getPose()
{
	return arm.getPose();
}

WMRA::JointValueSet ARM_module::getJointAngles()
{
	return arm.getJointAngles();
}

//void ARM_module::sendValues()
//{
//	arm.sendValues();
//}

bool ARM_module::isInitialized()
{
	return arm.isInitialized();
}

bool ARM_module::setInitialJointAngles(WMRA::JointValueSet& joints)
{
	return arm.setInitialJointAngles(joints);
}

void ARM_module::sendData(void* aArg)
{
	arm.sendData(aArg);
}

WMRA::JointValueSet ARM_module::getLastKnownJointPosition()
{
	return arm.getLastKnownJointPosition();
}

ARM_module::~ARM_module(void)
{
}

std::vector<double> ARM_module::getPosition()
{
	return arm.getPosition();
}

std::string ARM_module::command(std::string Command)
{
	return arm.command(Command);
}

std::vector<double> ARM_module::updateArmPosition()
{
	Q_arm_position = arm.updateWheelchairPosition();
	return Q_arm_position;
}