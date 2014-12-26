#include "driverDUOstereo.h"
#include <signal.h>


using namespace duoStereo_driver;


void sigIntHandler(int sig)
{
  	// Do some custom action.
  	// For example, publish a stop message to some other nodes.
  
	ROS_WARN("We Exited while loop-----------------------------");

	DUOStereoDriver::DestroyInstance();

  	// All the default sigint handler does is call shutdown()
  	ros::shutdown();
}


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "duo3d_node");

  	DUOStereoDriver& duoDriver = DUOStereoDriver::GetInstance();


  	signal(SIGINT, sigIntHandler);

  	if (duoDriver.initializeDUO())
  	{
  		duoDriver.startDUO();

  		ros::spin();
    }
    else
	{

		ROS_ERROR("Initialization failed. Exiting DUO Node.");
	}

	return 0;
}