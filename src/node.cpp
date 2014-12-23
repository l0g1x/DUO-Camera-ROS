#include "driverDUOstereo.h"

void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
	
}

int main(int argc, char **argv)
{

  	ros::init(argc, argv, "duo3d_node");
  	ros::NodeHandle node;
  	ros::NodeHandle priv_nh("~");
  	ros::NodeHandle camera_nh("duo3d_camera");

	DUOInstance 		_duoInstance;
	DUOResolutionInfo 	_duoResolutionInfo;

 //  	duoStereo_driver::DUOStereoDriver duoDriver(priv_nh, camera_nh);

 //  	if (duoDriver.InitializeDUO() == true)
 //  	{
 //  // 		StartDUO(_duoInstance, DUOCallback, NULL);
	//  //  	while(node.ok())
	//  //  	{
	//  //  		ros::spinOnce();
	//  //  	}
	//  //  	StopDUO(_duoInstance);
	// 	// CloseDUO(_duoInstance);
 //    }
 //    else
	// {
	// 	ROS_ERROR("Initialization failed. Exiting DUO Node.");
	// }


	return 0;
}