#ifndef DUOCamera_StereoDriver_h
#define DUOCamera_StereoDriver_h

#include <DUOLib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

namespace duoStereo_driver
{

class DUOStereoDriver
{
	 DUOStereoDriver(ros::NodeHandle priv_nh, ros::NodeHandle camera_nh);
	~DUOStereoDriver();
public:

	static DUOStereoDriver&	CreateInstance(ros::NodeHandle priv_nh, ros::NodeHandle camera_nh)
	{
		if( pSingleton == 0L )
			pSingleton = new DUOStereoDriver(priv_nh, camera_nh);

		return *pSingleton;
	}

	static DUOStereoDriver&	GetInstance(void)
	{
		return *pSingleton;
	}

	static void	DestroyInstance(void)
	{
		if(pSingleton != 0L)
		{
			delete pSingleton;
			pSingleton = NULL;
		}
	}

	bool initializeDUO();
	void shutdownDUO();

private:

	/*
	 * @brief
	 * Used for setting proper camera namespaces 
	 * (e.g. 'left/duo3d_camera/')
	 * (e.g. 'right/duo3d_camera/')
	 */
	static const int TWO_CAMERAS = 2;
	static const std::string CameraNames[TWO_CAMERAS]; // = {"left","right"};

	/*
	 * @brief Refer to DUO API Docs for these two
	 */
	DUOInstance 		_duoInstance;
	DUOResolutionInfo 	_duoResolutionInfo;

	/*
	 *	@brief DUO Device data we get during initialization
	 */
	char 	_duoDeviceName[260];
	char 	_duoDeviceSerialNumber[260];
	char 	_duoDeviceFirmwareVersion[260];
	char 	_duoDeviceFirmwareBuild[260];

	/*
	 *	@params for whether or not to use IMU and/or LED sequences
	 */
	bool 	_useDUO_Imu;
	bool	_useDUO_LEDs;


	/*
	 * @priv_nh: 		used for grabbing params for launch files configs
	 * @camera_nh:		passed into the two seperate camera_info_managers that will be
	 *					created
	 * @single_camera_nh:
	 *					nodehandle for each seperate camera
	 */
	ros::NodeHandle _priv_nh;
	ros::NodeHandle _camera_nh;
	ros::NodeHandle _single_camera_nh[TWO_CAMERAS];
	std::string		_camera_name; 	// = "duo3d_camera";

	/*
	 * @brief 
	 * Pass ImagePtr array of size 2, containing both the left and right camera
	 * images that we got from the DUOCallback function
	 */
	void publishImages(const sensor_msgs::ImagePtr image[TWO_CAMERAS]);

	/*
	 * @brief
	 * Two instances of image transport publishers; Left and Right publishers
	 */
	boost::shared_ptr<image_transport::ImageTransport> 	_it;
	image_transport::CameraPublisher 					_imagePub[TWO_CAMERAS];      

	/*
	 * @brief
	 * Create instance of CameraInfoManager for taking care of setting/getting
	 * calibration related information for camera info message.
	 */
	boost::shared_ptr<camera_info_manager::CameraInfoManager> _cinfo[TWO_CAMERAS];

	/* 
	 * @brief
	 * Check if the camera info matches the duo camera settings. If the camera settings
	 * change, then a warning will pop up saying you must recalibrate the stereo camera. 
	 */
	bool _calibrationMatches[TWO_CAMERAS];	 

	// Create transform broadcaster to transform image into the camera mount frame? 
	// Since the z axis is pointing into the image, but in the world z axis is up

	static DUOStereoDriver* pSingleton;
};

}

#endif
