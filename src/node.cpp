#include "driverDUOstereo.h"


int main(int argc, char **argv)
{

  	ros::init(argc, argv, "duo3d_node");
  	ros::NodeHandle node;
  	ros::NodeHandle priv_nh("~");
  	ros::NodeHandle camera_nh("duo3d_camera");

  	duoStereo_driver::DUOStereoDriver duoDriver(priv_nh, camera_nh);

  	if (duoDriver.initializeDUO())
  	{
	   	while(node.ok())
	  	{
	   		ros::spinOnce();
	  	}
	  	duoDriver.shutdownDUO();
    }
    else
	{
		ROS_ERROR("Initialization failed. Exiting DUO Node.");
	}

	return 0;
}