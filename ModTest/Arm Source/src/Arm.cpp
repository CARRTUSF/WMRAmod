


#include "stringUtility.h"
#include "kinematics.h"
#include "trajectory.h"
#include "jacobian.h"
#include "Utility.h"
#include "ConfigReader.h"
#include "WmraTypes.h"
#include "optimization.h"
#include "Arm.h"

#include <assert.h>

using namespace std;
using namespace math;
using namespace tthread;


Arm::Arm(){
	initialized = 0;
//	t = new thread(sendData,this);

	xyz_way.open("data/XYZ-way.csv");
	xyz_sent.open("data/XYZ-sent.csv");
	xyz_cont.open("data/XYZ-cont.csv");
	jointVel.open("data/jointVel.csv");

	//Initialize the gripper orientation wrt to arm base frame
	gripperInitRotDiff.SetSize(4,4);
	gripperInitRotDiff.Null();
	gripperInitRotDiff(1,0) = -1;
	gripperInitRotDiff(2,1) = -1;
	gripperInitRotDiff(0,2) = 1;
	gripperInitRotDiff(3,3) = 1;

   gripperOpen = true;
}

bool Arm::initialize(){
   controller.initialize();
   if(!setDefaults())
      return 0;
   else if(controller.isInitialized())  // If controller class is initialized
   {
      initialized = 1;
      readyPosition = getJointAngles();
	  t = new thread(running,this);
      return 1;
   }
   else
      return 0;
}

void Arm::running(void * aArg) {
	Arm* a = (Arm*)aArg;
	clock_t last_time, current_time;
	float dt;

	while(1)
	{
		/****Calculate dt****/
		clock_t current_time = clock();
		dt = (current_time - last_time)/CLOCKS_PER_SEC;
		last_time = current_time;
		/********************/


	}
}


void Arm::sendData( void * aArg){
	Arm* a = (Arm*)aArg;
	sending_udpsocket socket1( "localhost:6000" );
	sockstream output_sock( socket1 );
	receiving_udpsocket socket2( "localhost:6001" );
	sockstream read_sock( socket2 );

	WMRA::JointValueSet joints;

	if( !read_sock.is_open() ){
		cerr << "Could not open read socket for reading." << endl;
	}

	while(!a->isInitialized()) {Sleep(100);}

	string temp_str_buf;
	char temp_buf[200];
	while(true){
		do{
			getline( read_sock, temp_str_buf );
		} while( temp_str_buf.find("GETPOS")== string::npos ); // keep reading until message is received
		//send position
		if(a->isInitialized())
			joints = a->getJointAngles();
		output_sock << "POSITION " << joints[0] << " " << joints[1] << " " << joints[2] << " " << joints[3] << " " << joints[4] << " " << joints[5] << " " << joints[6] << endl; 
	}
}

bool Arm::openGripper(bool blocking){
   double position = controller.readPos(7) - 8;
   controller.positionControl(7,position);
   Sleep(7000);
   gripperOpen = true;
   return true;
}

bool Arm::closeGripper(bool blocking){
   double position = controller.readPos(7) + 8;
   controller.positionControl(7,position);
   Sleep(7000);
   gripperOpen = false;
   return true;
}

bool Arm::isGripperOpen(){
   return gripperOpen;
}

WMRA::JointValueSet Arm::getJointAngles(){
   WMRA::JointValueSet joints;
   std::vector<double> pos = controller.readPosAll();
   assert(pos.size() >= 7);
   for(int i = 0; i < joints.size(); i++){		// Sets the current location to a 1x8 vector		
      joints[i] = pos[i];
   }
   return joints;   
}

bool Arm::setInitialJointAngles(WMRA::JointValueSet& joints){

   for(int i = 0; i < joints.size();++i){		// set each joint position		
      controller.definePosition(i,joints[i]);
   }
   return true ;  
}

WMRA::Pose Arm::getPose(){
  
   vector<double> jointAngles = controller.readPosAll();
   Matrix pos = kinematics(jointAngles);
   //in pilot mode
   Matrix pilotTransform = pos  * (!gripperInitRotDiff); //calculated rotation part  
   for(int i =0; i < 4 ; i++){ // copy translation from original
      pilotTransform(i,3) = pos(i,3); 
   }
   WMRA::Pose pose = TransfomationToPose(pilotTransform);
   return pose;
}

bool Arm::autonomous(WMRA::Pose dest, WMRA::CordFrame cordFr, bool blocking){
   if(!controller.isInitialized()){
      return false;
   }  
   vector<double> startJointAng = controller.readPosAll();
   Matrix startLoc_T = kinematics(controller.readPosAll());
   Matrix destLoc_T(4,4);
   if(cordFr == WMRA::ARM_FRAME_ABS){
      destLoc_T = pose2TfMat(dest);
   }
   else if( cordFr == WMRA::ARM_FRAME_PILOT_MODE){
      //destLoc_T = pose2TfMat(dest);
      Matrix temp = pose2TfMat(dest); // convert to rot matrix
      /* compensate for the gripper orintation difference compared to arm origin */
      destLoc_T =  temp * gripperInitRotDiff;  
      cout << destLoc_T << endl;
      /* set x, y, z values of the matrix*/
      destLoc_T(0,3) = dest.x;
      destLoc_T(1,3) = dest.y;
      destLoc_T(2,3) = dest.z;      
   }
   else if( cordFr == WMRA::ARM_FRAME_REL){
      destLoc_T = startLoc_T * WMRA_rotz(dest.yaw)*WMRA_roty(dest.pitch)*WMRA_rotx(dest.roll);;
      destLoc_T(0,3) = startLoc_T(0,3)+ dest.x;
      destLoc_T(1,3) = startLoc_T(1,3)+ dest.y;
      destLoc_T(2,3) = startLoc_T(2,3)+ dest.z;
      cout << destLoc_T << endl;
   }
   else if (cordFr == WMRA::GRIPPER_FRAME_REL){
      destLoc_T = startLoc_T * pose2TfMat(dest);
   }
   else{ // if an invalid cord frame is given, move in arm base absolute
      destLoc_T = pose2TfMat(dest);
   }
   /**call autonomousMove with start and dest transformation matrices **/
   return autonomousMove(startLoc_T, destLoc_T, blocking);
}

bool Arm::teleoperation(WMRA::Pose deltaPose){
		double dt_mod = this->dt;
		return this->teleoperation(deltaPose, dt_mod);
}

bool Arm::teleoperation(WMRA::Pose deltaPose, double deltaTime){

   if(controller.isInitialized())	// If WMRA controller connection has been initialized start loop
   {
		if(controller.getMotorMode() != 2){
			controller.setMotorMode(MotorController::VELOCITY);
		}
		
		KinematicOptimizer opt; // WMRA kinematics optimizer functionality
		
		Matrix Ta(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T67(4,4);
		Matrix Joa;
		double detJoa;
		std::vector<double> currJointAng(7), delta(8), speeds(8);

		/*current joint angles */
		std::vector<double> startJointAng = controller.readPosAll() ;		
		
		/*set start and destination transformation matrix based on current position and delta pose */
		Matrix startPosTF =  kinematics(startJointAng);
		Matrix destPosTF = startPosTF * pose2TfMat(deltaPose);

		// Calculating the transformation matrix of each joint
		kinematics(startJointAng,Ta,T01,T12,T23,T34,T45,T56,T67);
		
		// Calculating the 6X7 Jacobian of the arm in frame 0:
		WMRA_J07(T01, T12, T23, T34, T45, T56, T67, Joa, detJoa);

		WMRA_delta(delta, startPosTF , destPosTF);

		Matrix jointAng_Mat = opt.WMRA_Opt2(Joa, detJoa, delta, startJointAng, deltaTime);

		for(int j = 0; j < 7; j++){
			startJointAng[j] = jointAng_Mat(j,0);
		}

		for(int k = 0; k < startJointAng.size(); k++){
			speeds[k] = (startJointAng[k])/deltaTime; 
			//speeds[k] = abs(startJointAng[k])/deltaTime; // #DEBUG - should this be the absolute value
		}

		jointVel << speeds[0] << "," << speeds[1] << "," << speeds[2] << "," << speeds[3] << "," 
		<< speeds[4] << "," << speeds[5] << "," << speeds[6] << endl;
		
		controller.sendJog(speeds);
		
		return true;
	}
	else{
		return false;
	}
	return true;
}

bool Arm::teleoperation(WMRA::Pose dest, WMRA::CordFrame cordFr){
   if(!controller.isInitialized()){
      return false;
   }  
   vector<double> startJointAng = controller.readPosAll();
   Matrix startLoc_T = kinematics(controller.readPosAll());
   Matrix destLoc_T(4,4);
   if(cordFr == WMRA::ARM_FRAME_ABS){
      destLoc_T = pose2TfMat(dest);
   }
   else if( cordFr == WMRA::ARM_FRAME_PILOT_MODE){
      //destLoc_T = pose2TfMat(dest);
      Matrix temp = pose2TfMat(dest); // convert to rot matrix
      /* compensate for the gripper orintation difference compared to arm origin */
      destLoc_T =  temp * gripperInitRotDiff;  
      cout << destLoc_T << endl;
      /* set x, y, z values of the matrix*/
      destLoc_T(0,3) = dest.x;
      destLoc_T(1,3) = dest.y;
      destLoc_T(2,3) = dest.z;      
   }
   else if( cordFr == WMRA::ARM_FRAME_REL){
      destLoc_T = startLoc_T * WMRA_rotz(dest.yaw)*WMRA_roty(dest.pitch)*WMRA_rotx(dest.roll);;
      destLoc_T(0,3) = startLoc_T(0,3)+ dest.x;
      destLoc_T(1,3) = startLoc_T(1,3)+ dest.y;
      destLoc_T(2,3) = startLoc_T(2,3)+ dest.z;
      cout << destLoc_T << endl;
   }
   else if (cordFr == WMRA::GRIPPER_FRAME_REL){
      destLoc_T = startLoc_T * pose2TfMat(dest);

   }
   else{ // if an invalid cord frame is given, move in arm base absolute
      destLoc_T = pose2TfMat(dest);
   }
   /**call autonomousMove with start and dest transformation matrices **/
   return teleoperationMove(startLoc_T, destLoc_T);
}

bool Arm::motionComplete() {
	return controller.motionFinished();
}

bool Arm::moveJoint(int jointNum, double angle, int ref)
{
	controller.setMotorMode(controller.POS_CONTROL);
	long jointEnc;

	if(ref==0) // absolute
	{
		controller.positionControl(jointNum, angle);
		return 1;
	}
	else if(ref==1) // relative
	{
		double tempJointVal = controller.readPos(jointNum);
		tempJointVal = tempJointVal + angle;
		controller.positionControl(jointNum, tempJointVal);
		return 1;
	}

	Sleep(1000);
	controller.setMotorMode(controller.LINEAR);
	return 0;
}

bool Arm::stop(){
	return controller.Stop();
}

bool Arm::autonomousMove(Matrix start, Matrix dest, bool blocking){
   /** calculate angular distance **/
   Matrix startRot(3,3),destRot(3,3);
   for ( int i=0 ; i < 3 ; i++ ) {  //deep copy rotation portion
      for ( int j = 0 ; j < 3 ; j++ ) {
         startRot(i,j)=start(i,j);
         destRot(i,j)=dest(i,j);
      }
   }
   Matrix R = (~startRot) * destRot; //delta rotation from start to dest
   double singleAngleDist = atan2(sqrt(pow((R(2,1)-R(1,2)),2)+pow((R(0,2)-R(2,0)),2)+pow((R(1,0)-R(0,1)),2)),(R(0,0)+R(1,1)+R(2,2)-1));
   double angularDist = singleAngleDist;
   int angularPoints = ceil(angularDist/(Arm::maxAngularVelocity * Arm::dt));
   /** calculate linear distance**/
   double linearDist = sqrt(pow(dest(0,3)-start(0,3),2) + pow(dest(1,3)-start(1,3),2) + pow(dest(2,3)-start(2,3),2));
   int linearPoints = ceil(linearDist/( Arm::control_velocity * Arm::dt));
   /** calculate number of waypoints **/
   double totalTime;
   int numWayPoints;
   if( linearPoints > angularPoints ){
      numWayPoints = linearPoints;
      totalTime = linearDist / Arm::control_velocity;         
   }
   else{
      numWayPoints = angularPoints;
      totalTime = angularDist / Arm::maxAngularVelocity ;
   }
   dt_mod = totalTime/numWayPoints;
   /** get trajectory **/
   std::vector<Matrix> wayPoints = WMRA_traj(3, start, dest, numWayPoints+1);


   if(controller.isInitialized())	// If WMRA controller connection has been initialized start loop
   { 
      KinematicOptimizer opt; // WMRA kinematics optimizer functionality
      Matrix Ta(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T67(4,4);
      Matrix Joa;
      double detJoa;
      std::vector<double> currJointAng(7), delta(8), speeds(7);
      //get the initial Arm position
      std::vector<double> startJointAng = controller.readPosAll() ;
      //set previous position to current before the loop
      std::vector<double> prevJointAng = startJointAng;
      Matrix prevPosTF =  kinematics(startJointAng);
      Matrix currPosTF; 
      Matrix jointAng_Mat;

      //xyz_sent << startLoc_T(0,3) << "," << startLoc_T(1,3) << "," << startLoc_T(2,3) << endl;
      //xyz_way << startLoc_T(0,3) << "," << startLoc_T(1,3) << "," << startLoc_T(2,3) << endl;
      

      for(int i = 1 ; i < numWayPoints +1; i++)
      {			
         kinematics(prevJointAng,Ta,T01,T12,T23,T34,T45,T56,T67);	

         // Calculating the 6X7 Jacobian of the arm in frame 0:
         WMRA_J07(T01, T12, T23, T34, T45, T56, T67, Joa, detJoa);

         //#debug need to transform waypoints to arm base see matlab code WMRA_main.m line 346
         currPosTF = wayPoints[i];
         xyz_way << currPosTF(0,3) << "," << currPosTF(1,3) << "," << currPosTF(2,3) << endl;

         WMRA_delta(delta, prevPosTF , currPosTF);

         jointAng_Mat = opt.WMRA_Opt2(Joa, detJoa, delta, prevJointAng, dt_mod);

         for(int j = 0; j < 7; j++){
            currJointAng[j] = jointAng_Mat(j,0);
            prevJointAng[j] += currJointAng[j];
         }

         //**debug**//
         Matrix test_T = kinematics(prevJointAng,Ta,T01,T12,T23,T34,T45,T56,T67);	
         xyz_sent << test_T(0,3) << "," << test_T(1,3) << "," << test_T(2,3) << endl;
         //*******//

         for(int k = 0; k < currJointAng.size(); k++){
            speeds[k] = abs(currJointAng[k])/dt_mod;
         }

         jointVel << speeds[0] << "," << speeds[1] << "," << speeds[2] << "," << speeds[3] << "," 
            << speeds[4] << "," << speeds[5] << "," << speeds[6] << endl;

         
         controller.addLinearMotionSegment(currJointAng, speeds);
		 if(i == 1)controller.beginLI();
         controller.endLIseq();

         //prevPosTF = currPosTF;
         prevPosTF = kinematics(prevJointAng);
      }

     
      Matrix debugPos_T = kinematics(controller.readPosAll());
      xyz_cont << debugPos_T(0,3) << "," << debugPos_T(1,3) << "," << debugPos_T(2,3) << endl;

      controller.beginLI();
      controller.endLIseq();
	  cout << ">> All motion commands sent" << endl;
	  /* wait for motion to complete */
	  if(blocking){
		  while(!motionComplete()){
			  getJointAngles();
			  Sleep(30);
		  }
		  cout << ">> Motion complete" << endl;
	  }
	  
    /*  for(int k = 0; k < (numWayPoints+10); k++){
         debugPos_T = kinematics(controller.readPosAll());
         xyz_cont << debugPos_T(0,3) << "," << debugPos_T(1,3) << "," << debugPos_T(2,3) << endl;        
         Sleep(1000* dt_mod);
      }*/
	  
   }
   else{
      return false;
   }
   return true;
}

bool Arm::teleoperationMove(Matrix start, Matrix dest){

   dt_mod = Arm::dt;

   /** get trajectory **/
   std::vector<Matrix> wayPoints = WMRA_traj(3, start, dest, 2);
   
   if(controller.isInitialized())	// If WMRA controller connection has been initialized start loop
   { 
		KinematicOptimizer opt; // WMRA kinematics optimizer functionality
		Matrix Ta(4,4), T01(4,4), T12(4,4), T23(4,4), T34(4,4), T45(4,4), T56(4,4), T67(4,4);
		Matrix Joa;
		double detJoa;
		std::vector<double> currJointAng(7), delta(8), speeds(7);
		//get the initial Arm position
		std::vector<double> startJointAng = controller.readPosAll() ;
		//set previous position to current before the loop
		std::vector<double> prevJointAng = startJointAng;
		Matrix prevPosTF =  kinematics(startJointAng);
		Matrix currPosTF; 
		Matrix jointAng_Mat;
	
		kinematics(prevJointAng,Ta,T01,T12,T23,T34,T45,T56,T67);	

		// Calculating the 6X7 Jacobian of the arm in frame 0:
		WMRA_J07(T01, T12, T23, T34, T45, T56, T67, Joa, detJoa);

		//#debug need to transform waypoints to arm base see matlab code WMRA_main.m line 346
		currPosTF = wayPoints[1];
		xyz_way << currPosTF(0,3) << "," << currPosTF(1,3) << "," << currPosTF(2,3) << endl;

		WMRA_delta(delta, prevPosTF , currPosTF);

		jointAng_Mat = opt.WMRA_Opt2(Joa, detJoa, delta, prevJointAng, dt_mod);

		for(int j = 0; j < 7; j++){
			currJointAng[j] = jointAng_Mat(j,0);
			prevJointAng[j] += currJointAng[j];
		}

		for(int k = 0; k < currJointAng.size(); k++){
			speeds[k] = abs(currJointAng[k])/dt_mod;
		}

		jointVel << speeds[0] << "," << speeds[1] << "," << speeds[2] << "," << speeds[3] << "," 
		<< speeds[4] << "," << speeds[5] << "," << speeds[6] << endl;

		controller.setJointLimits();
		controller.sendJog(speeds);
	}
	else{
		return false;
	}
	return true;
}

void Arm::closeDebug(){
   xyz_way.close();
   xyz_sent.close();
   xyz_cont.close();
   jointVel.close();
}

WMRA::JointValueSet Arm::getLastKnownJointPosition(){
   WMRA::JointValueSet joints;
   std::vector<double> pos = controller.getLastKnownPos();
   for(int i = 0; i < joints.size(); i++){		// Sets the current location to a 1x8 vector		
      joints[i] = pos[i];
   }
   return joints; 
}

vector<double> Arm::getPosition()
{
	return controller.readPosAll();
   //return controller.getLastKnownPos();  
}

vector<double> Arm::updateWheelchairPosition()
{
	vector<double> temp =  getPosition();
	for(int i = 0; i<8; i++)
	{
		current_arm_position[i] = temp[i];
		Q_arm_position[i] = initial_Arm_position[i] + current_arm_position[i];
	}
	return Q_arm_position;
}

bool Arm::toReady(bool blocking)
{
	controller.Stop();
	Sleep(1000);
	if(controller.getMotorMode() != 1){
		controller.setMotorMode(MotorController::POS_CONTROL);
		controller.setMotorMode(MotorController::LINEAR);
	}
   double angles[7] = {M_PI/2, M_PI/2,0, M_PI/2,M_PI/2,M_PI/3,0};
   vector<double> readyAng;
   
   vector<double> speeds;
   readyAng.push_back(1.570796327-controller.readPos(0));
   readyAng.push_back(1.570796327-controller.readPos(1));
   readyAng.push_back(0.0-controller.readPos(2));
   readyAng.push_back(1.570796327-controller.readPos(3));
   readyAng.push_back(1.570796327-controller.readPos(4));
   readyAng.push_back(1.0471975513-controller.readPos(5));
   readyAng.push_back(0.0-controller.readPos(6));
   speeds.push_back(0.02);
   speeds.push_back(0.02);
   speeds.push_back(0.02);
   speeds.push_back(0.02);
   speeds.push_back(0.02);
   speeds.push_back(0.02);
   speeds.push_back(0.02);
   controller.addLinearMotionSegment(readyAng, speeds);
   controller.beginLI();
   controller.endLIseq();
   cout << "Moving to ready position" << endl;
   return true;
}

bool Arm::ready2Park(bool blocking)
{
	controller.positionControl(3, degToRad(180)); //joint 4 moved down, parallel with link 3
	Sleep(10000);
	controller.positionControl(0, 0.0); //joint 1 moved back from ready by 90 degrees.
	Sleep(10000);

	return 1;
}

bool Arm::park2Ready(bool blocking)
{
	controller.positionControl(0, degToRad(90)); //joint 4 moved down, parallel with link 3
	Sleep(10000);
	controller.positionControl(3, degToRad(90)); //joint 1 moved back from ready by 90 degrees.
	Sleep(10000);

	return 1;
}

bool Arm::isInitialized()
{
	return Arm::initialized;
}

bool Arm::setDefaults()
{
   initial_Arm_position.resize(8);
   current_arm_position.resize(8);
   Q_arm_position.resize(8);
   Arm_link.resize(7);

   ConfigReader reader;
   reader.parseFile("WMRA_arm_settings.conf");
   reader.setSection("ARM_DEFAULTS");

   if(reader.keyPresent("dt"))
   {
      Arm::dt = reader.getDouble("dt");
      //cout << "dt: " << Arm::dt << endl;
   }
   else
   {
      cout << "'dt' default not found" << endl;
      return 0;
   }

   if(reader.keyPresent("control_velocity"))
   {
      Arm::control_velocity = reader.getInt("control_velocity");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'control_velocity' default not found" << endl;
      return 0;
   }

   if(reader.keyPresent("maxAngularVelocity"))
   {
      Arm::maxAngularVelocity = reader.getDouble("maxAngularVelocity");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'maxAngularVelocity' default not found" << endl;
      return 0;
   }

   if(reader.keyPresent("initial_arm_1"))
   {
      Arm::initial_Arm_position[0] = reader.getDouble("initial_arm_1");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'initial_arm_1' default not found" << endl;
      return 0;
   }

   if(reader.keyPresent("initial_arm_2"))
   {
      Arm::initial_Arm_position[1] = reader.getDouble("initial_arm_2");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'initial_arm_2' default not found" << endl;
      return 0;
   }

   if(reader.keyPresent("initial_arm_3"))
   {
      Arm::initial_Arm_position[2] = reader.getDouble("initial_arm_3");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'initial_arm_3' default not found" << endl;
      return 0;
   }

   if(reader.keyPresent("initial_arm_4"))
   {
      Arm::initial_Arm_position[3] = reader.getDouble("initial_arm_4");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'initial_arm_4' default not found" << endl;
      return 0;
   }
   
   if(reader.keyPresent("initial_arm_5"))
   {
      Arm::initial_Arm_position[4] = reader.getDouble("initial_arm_5");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'initial_arm_5' default not found" << endl;
      return 0;
   }
   
   if(reader.keyPresent("initial_arm_6"))
   {
      Arm::initial_Arm_position[5] = reader.getDouble("initial_arm_6");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'initial_arm_6' default not found" << endl;
      return 0;
   }
   
   if(reader.keyPresent("initial_arm_7"))
   {
      Arm::initial_Arm_position[6] = reader.getDouble("initial_arm_7");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'initial_arm_7' default not found" << endl;
      return 0;
   }
   
   if(reader.keyPresent("initial_arm_8"))
   {
      Arm::initial_Arm_position[7] = reader.getDouble("initial_arm_8");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'initial_arm_8' default not found" << endl;
      return 0;
   }
      
   if(reader.keyPresent("link_1"))
   {
      Arm::Arm_link[0] = reader.getDouble("link_1");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'link_1' default not found" << endl;
      return 0;
   }
         
   if(reader.keyPresent("link_2"))
   {
      Arm::Arm_link[1] = reader.getDouble("link_2");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'link_2' default not found" << endl;
      return 0;
   }
         
   if(reader.keyPresent("link_3"))
   {
      Arm::Arm_link[2] = reader.getDouble("link_3");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'link_3' default not found" << endl;
      return 0;
   }
         
   if(reader.keyPresent("link_4"))
   {
      Arm::Arm_link[3] = reader.getDouble("link_4");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'link_4' default not found" << endl;
      return 0;
   }
         
   if(reader.keyPresent("link_5"))
   {
      Arm::Arm_link[4] = reader.getDouble("link_5");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'link_5' default not found" << endl;
      return 0;
   }
         
   if(reader.keyPresent("link_6"))
   {
      Arm::Arm_link[5] = reader.getDouble("link_6");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'link_6' default not found" << endl;
      return 0;
   }

   if(reader.keyPresent("link_7"))
   {
      Arm::Arm_link[6] = reader.getDouble("link_7");
      //cout << "control_velocity: " << Arm::control_velocity << endl;
   }
   else
   {
      cout << "'link_7' default not found" << endl;
      return 0;
   }

   current_arm_position.resize(8);
   for(int i = 0; i<8; i++)
   {
	   current_arm_position[i] = 0.0;
	   Q_arm_position[i] = initial_Arm_position[i] + current_arm_position[i];
   }


   return 1;
}

int Arm::joint_limit_avoidance(int joint_name, int encoder_count, int speed)
{
	const int Joint_A_encoder_min = -2240640;
	const int Joint_A_encoder_max = 1712130;
	const int Joint_B_encoder_min = -2384620;
	const int Joint_B_encoder_max = 900000;
	const int Joint_C_encoder_min = -500000;
	const int Joint_C_encoder_max = 800000;
	const int Joint_D_encoder_min = -1200000;
	const int Joint_D_encoder_max = 800000;
	const int Joint_E_encoder_min = -1000000;
	const int Joint_E_encoder_max = 1557420;
	const int Joint_F_encoder_min = -384987;
	const int Joint_F_encoder_max = 1543350;
	const int Joint_G_encoder_min = -1387870;
	const int Joint_G_encoder_max = 1100000;
	const int Joint_H_encoder_min = -600000;
	const int Joint_H_encoder_max = 600000;
	switch (joint_name)
	{
	case 1:
		if (encoder_count>Joint_A_encoder_max && speed>0)
		{
			return 0;
		}
		else if (encoder_count<Joint_A_encoder_min && speed<0)
		{
			return 0;
		}
		else
		{
			return speed;
		}
		break;
	case 2:
		if (encoder_count>Joint_B_encoder_max && speed>0)
		{
			return 0;
		}
		else if (encoder_count<Joint_B_encoder_min && speed<0)
		{
			return 0;
		}
		else
		{
			return speed;
		}
		break;
	case 3:
		if (encoder_count>Joint_C_encoder_max && speed>0)
		{
			return 0;
		}
		else if (encoder_count<Joint_C_encoder_min && speed<0)
		{
			return 0;
		}
		else
		{
			return speed;
		}
		break;
	case 4:
		if (encoder_count>Joint_D_encoder_max && speed>0)
		{
			return 0;
		}
		else if (encoder_count<Joint_D_encoder_min && speed<0)
		{
			return 0;
		}
		else
		{
			return speed;
		}
		break;
	case 5:
		if (encoder_count>Joint_E_encoder_max && speed>0)
		{
			return 0;
		}
		else if (encoder_count<Joint_E_encoder_min && speed<0)
		{
			return 0;
		}
		else
		{
			return speed;
		}
		break;
	case 6:
		if (encoder_count>Joint_F_encoder_max && speed>0)
		{
			return 0;
		}
		else if (encoder_count<Joint_F_encoder_min && speed<0)
		{
			return 0;
		}
		else
		{
			return speed;
		}
		break;
	case 7:
		if (encoder_count>Joint_G_encoder_max && speed>0)
		{
			return 0;
		}
		else if (encoder_count<Joint_G_encoder_min && speed<0)
		{
			return 0;
		}
		else
		{
			return speed;
		}
		break;
	case 8:
		if (encoder_count>Joint_H_encoder_max && speed>0)
		{
			return 0;
		}
		else if (encoder_count<Joint_H_encoder_min && speed<0)
		{
			return 0;
		}
		else
		{
			return speed;
		}
		break;
	default:return 0; break;
	}
}

int Arm::joint_max_speed(int joint_name)
{
	const int joint_A_max_speed = 100000;
	const int joint_B_max_speed = 200000;
	const int joint_C_max_speed = 200000;
	const int joint_D_max_speed = 200000;
	const int joint_E_max_speed = 200000;
	const int joint_F_max_speed = 200000;
	const int joint_G_max_speed = 150000;
	const int joint_H_max_speed = 250000;

	switch (joint_name)
	{
	case 1:return joint_A_max_speed; break;
	case 2:return joint_B_max_speed; break;
	case 3:return joint_C_max_speed; break;
	case 4:return joint_D_max_speed; break;
	case 5:return joint_E_max_speed; break;
	case 6:return joint_F_max_speed; break;
	case 7:return joint_G_max_speed; break;
	case 8:return joint_H_max_speed; break;
	default:return 0; break;
	}
}

int Arm::joint_speed_limit(int joint_name, int speed)
{
	if(abs(speed)>joint_max_speed(joint_name))
	{
		if(speed>0)
		{
			return joint_max_speed(joint_name);
		}
		else
		{
			return -joint_max_speed(joint_name);
		}
	}
	else
	{
		return speed;
	}
}

std::string Arm::command(std::string Command)
{
	return controller.command(Command);
}