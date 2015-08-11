////////////////////////////////////////////////////////////////////
//
//	A ROS driver for the DUO3D Camera.
//	Author: Krystian Gebis
//	File:	driverDUOstereo.cpp
//
////////////////////////////////////////////////////////////////////

#ifndef DUOCamera_Dense3DDriver_h
#define DUOCamera_Dense3DDriver_h

#include <DUOLib.h>
#include <Dense3D.h>
#include "driverDUOstereo.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>
#include <duo3d_ros/Dense3DConfig.h>


using namespace duoStereo_driver;

namespace duoDense3D_driver
{

class DUODense3DDriver
{

public:

	/*
	 *	@brief
	 *	Singleton Implementation to allow for only a single instance of this
	 *	class.
	 *	-If GetInstance() is called more then once, it will return a pointer
	 *	 to a already initialize (non-NULL) pSingleton object.
	 *	-If GetInstance() is called for the first time, initialize the pSingleton
	 *	 variable to a new DUOStereoDriver object.
	 *
	 *	@return
	 *	A pointer to the pSingleton private member object.
	 */
	static DUODense3DDriver&	GetInstance(void)
	{
		if( pSingleton == 0L )
			pSingleton = new DUODense3DDriver();

		return *pSingleton;
	}

	/*
	 *	@brief
	 *	Check if pSingleton object is not null, and if it is not null, call the
	 *	shutdownDUO() function FIRST, and then delete the pSingleton object.
	 *
	 */
	static void	DestroyInstance(void)
	{
		if(pSingleton != 0L)
		{
			pSingleton->shutdownDense3D();

			delete pSingleton;
			pSingleton = NULL;
		}
	}

	/*
	 *	@brief
	 *	initializeDUO(): 	Check if roslaunch parameters are valid for use with
	 *						DUO Camera. If they are, then open a DUO connection
	 *						with OpenDUO() DUO API function call.
	 *	startDUO():			Simply calls the DUO API StartDUO() function call.
	 *	shutdownDUO():		Using the DUO API function calls to properly end connection
 	 * 						with DUO camera. This should ONLY be called, if the ros node
 	 * 						receives a shutdown signal; so this is called in the
 	 * 						DestroyInstance() if the pSingleton instance is not NULL.
	 */
	bool initializeDense3D(void);
	void startDense3D(void);
	void shutdownDense3D(void);
	void setup(void);

	/*
	 *	@brief
	 *	Used in the DUOCallback function for determine which image array index
	 *	to store the respective data in.
	 */

private:

	/*
	 *	@brief
	 *	Constructor and Destructor are made private so that you can only get one instance
	 *	of this class through the GetInstance() function call, after this DUOStereoDriver
	 *	constructor is called for the first time.
	 */
	DUODense3DDriver(void);
	~DUODense3DDriver(void);


	int _speckleWindowSize;  	// 0 - 256
	int _speckleRange;			// 0 - 16
	int _uniquenessRatio;		// 0 - 100
	int _preFilterCap;			// 0 - 256
	int _p1;					// 0 - 3200
	int _p2;					// 0 - 3200
	int _sadWindowSize;		// 2 - 10
	int _numDisparities;		// 2 - 16




	/*
	 *	@brief Refer to DUO API Docs for these two
	 */
	Dense3DInstance		_dense3dInstance;
	DUOInstance 		_duoInstance;
	DUOResolutionInfo 	_duoResolutionInfo;
	DUOStereoDriver&	_duoStereoDriver;


	/*
	 * 	@priv_nh: 		used for grabbing params for launch files configs
	 */
	ros::NodeHandle _priv_nh;

	dynamic_reconfigure::Server<duo3d_ros::Dense3DConfig> 				_dynamicServer;
	dynamic_reconfigure::Server<duo3d_ros::Dense3DConfig>::CallbackType _serverCbType;

	void dynamicCallback(duo3d_ros::Dense3DConfig &config, uint32_t level);

	void setParameters();

	static DUODense3DDriver* pSingleton;
};

}

#endif
