////////////////////////////////////////////////////////////////////
//
//	A ROS driver for the DUO3D Camera.
//	Author: Krystian Gebis
//	File:	driverDUOstereo.cpp
//
////////////////////////////////////////////////////////////////////


#include "driverDUODense3D.h"
#include <signal.h>

using namespace duoDense3D_driver;


/*
 * 	@brief
 * 	Have to implement this custom SIGINT Handler in order to properly
 * 	shutdown the DUO camera and call the shutdownDUO() function
 * 	to terminate the connection between the computer and camera.
 * 	http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
 */
void sigIntHandler(int sig)
{
    ROS_DEBUG("--> SIGINT Handler called <--");

    //Destory our pSingleton Instance that we created.
    DUODense3DDriver::DestroyInstance();

    // Tell the ros node that we want to shutdown, so we receive a
    // clean exit
    ros::shutdown();
}


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "duodense3d_node");

  	/*
  	 *	@brief
  	 *	Singleton implementation; Calling the GetInstance() function, which will
  	 *	return us the pointer to the pSingleton member, so that we always only use
  	 * 	one instance of the DUOStereoDriver class, and then can normally call public
  	 *	member functions later.
  	 */
  	DUODense3DDriver& dense3dDriver = DUODense3DDriver::GetInstance();


	/*
	 *	@brief
	 *	Must call signal() after first NodeHandle call (we have multiple NodeHandler's
	 *	being initialized in the contructor, which is fine). This listen to, for example,
	 * 	a 'ctrl+c' shutdown signal, and then the sigIntHandler callback is invoked.
	 */
  	signal(SIGINT, sigIntHandler);

  	/*
  	 *	@brief
  	 *	First see if we can successfully initialize the DUO, then start the DUO
  	 * 	to capture frames from camera. Once the camera is started, ros::spin()
  	 *	is called so that the NodeHandle publishers have a chance to be invoked
  	 * 	and send the images to their topics.
  	 */
  	if (dense3dDriver.initializeDense3D())
  	{
  		dense3dDriver.setup();
  		dense3dDriver.startDense3D();

  		ros::spin();
    }
    else
	{

		ROS_ERROR("Initialization failed. Exiting DUO Dense3D Node.");
	}

	return 0;
}
