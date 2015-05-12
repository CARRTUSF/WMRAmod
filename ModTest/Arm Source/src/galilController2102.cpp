#include "galilController2102.h"
#include "SockStream.h"
#include "ConfigReader.h"

using namespace std;

/************************
//
// Galil Controller Class
//
************************/

client_tcpsocket galilController2102::sock;

galilController2102::galilController2102(){
	debugFile.open("data/GalilDebug.log");
	IP = "192.168.1.22";
	simulation = 0;
	debug = 0;
	initialized = false;
}

galilController2102::~galilController2102(){
	debugFile.close();
	sock.close();
}

bool galilController2102::initialize()
{
	
	if(simulation)
	{
		cout << "Simulation Mode" << endl;
		initialized = true;
	}
	else
	{
		initialized = initializeSocket(galilController2102::IP);
	}
	if(!initialized)
	{
		cout << "Error: Failed to initialize Galil Controller" << endl;
		return 0;
	}
	else
	{
		if(simulation)
		{
			if(isDebug()) cout << "Simulation Initialized" << endl;
			return 1;
		}
		else
		{
			if(isDebug()) cout << "Galil: Initialized" << endl;
			return 1;
		}
	}
}

bool galilController2102::isInitialized() // return initialized
{
	return initialized;
}

bool galilController2102::isSimulated() // return simulation
{
	return simulation;
}
bool galilController2102::isDebug() // return simulation
{
	return debug;
}

std::string galilController2102::command(std::string Command)
{
	if(simulation)
	{
		return "0";
	}
	else
	{
		if(debug)
			debugFile << Command << endl;
		char com[300];
		char ret[300];
		std::string c = Command + "\r";
		strcpy(com, c.c_str());
		commandGalil(com, ret, sizeof(ret));
		std::string ret_str(ret);
		return ret_str;
	}
}


bool galilController2102::initializeSocket(std::string IP)
{
	const char * c = IP.c_str();
	sock.open(c,23);
	try{
		galilController2102::sock.connected(); // #DEBUG - breaks if trying to setup sock while WMRA is off
	}
	catch (...)
	{
		return 0;
	}
	return 1;
}

int galilController2102::commandGalil(char* Command, char* Response, int ResponseSize) //returns the number of bytes read
{
	//command() sends an ASCII Command (e.g. "TPX") to the controller and retrieves a Response (e.g. "123\r\n:").
	//The size of Response should be supplied as ResponseSize so that unallocated memory is not overwritten.
	//If you statically allocate your response buffer (e.g. char buff[100]) use sizeof(buff).
	char acPartialResponse[512] = {0}; //buffer to contain partial responses (which will be concatenated together to form the final response)
	int iPartialBytesRead = 0; //number of bytes read each time through the loop
	int iTotalBytesRead = 0;   //the total number of bytes read.  Can't exceed ResponseSize.  
	Response[0] = 0; //set response to null string    
	sock.write(Command, strlen(Command));

	//keep reading until we (a) get a colon (b) get a question mark (c) fill up the callers Response buffer
	while(1)
	{
		iPartialBytesRead = sock.read(acPartialResponse, sizeof(acPartialResponse)); //read some characters

		if(iPartialBytesRead <= 0)   //nothing read, keep reading until :
			continue;
		else if(iTotalBytesRead + iPartialBytesRead > ResponseSize) //get out of the loop if we will fill up the caller's buffer, iPartialBytesRead >= 1
			break;
		else {
			strncat(Response, acPartialResponse, iPartialBytesRead); //add the partial response to the full response.  Response is null terminated
			iTotalBytesRead += iPartialBytesRead; //tally up the total number of bytes read
			if (acPartialResponse[iPartialBytesRead - 1] == ':' || acPartialResponse[iPartialBytesRead - 1] == '?') //got a colon, iPartialBytesRead >= 1
				break;
		}
	}
	return(iTotalBytesRead);
}
