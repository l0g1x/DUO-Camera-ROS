////////////////////////////////////////////////////////////////////
//
//	A ROS driver for the DUO3D Camera.
//	Author: Krystian Gebis
//	File:	driverDUOstereo.cpp
//
////////////////////////////////////////////////////////////////////
 
#ifndef DUOCamera_StereoDriver_h
#define DUOCamera_StereoDriver_h

#include <DUOLib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <dynamic_reconfigure/server.h>
#include <duo3d_ros/DuoConfig.h>


namespace duoStereo_driver
{

class DUOStereoDriver
{

public:

	/*
	 * 	@brief
	 * 	This outside DUO API function ONLY, can access this DUOStereoDriver class 
	 * 	private members since it is listed as a friend to this class. 
	 * 	Be careful when using this. 
	 */
	friend void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData);

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
	static DUOStereoDriver&	GetInstance(void)
	{
		if( pSingleton == 0L )
			pSingleton = new DUOStereoDriver();

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
			pSingleton->shutdownDUO();

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
	bool initializeDUO(void);
	void startDUO(void);
	void shutdownDUO(void);
	void setup(void);

	/*
	 *	@brief
	 *	Used in the DUOCallback function for determine which image array index
	 *	to store the respective data in. 
	 */
	static const int TWO_CAMERAS	= 2;
	static const int LEFT_CAM 		= 0;
	static const int RIGHT_CAM		= 1;

private:

	/*
	 *	@brief
	 *	Constructor and Destructor are made private so that you can only get one instance
	 *	of this class through the GetInstance() function call, after this DUOStereoDriver
	 *	constructor is called for the first time. 
	 */
	DUOStereoDriver(void);
	~DUOStereoDriver(void);

	/*
	 * 	@brief
	 * 	Used for setting proper camera namespaces 
	 * 	(e.g. 'left/duo3d_camera/')
	 * 	(e.g. 'right/duo3d_camera/')
	 */
	static const std::string CameraNames[TWO_CAMERAS]; // = {"left","right"};

	/*
	 *	@brief Refer to DUO API Docs for these two
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

	double 	_duoExposure;
	double	_duoGain;
	double 	_duoLEDLevel;
	int 	_duoCameraSwap;
	int 	_duoHorizontalFlip;
	int 	_duoVerticalFlip;

	/*
	 *	@params for whether or not to use IMU and/or LED sequences
	 */
	bool 	_useDUO_Imu;
	bool	_useDUO_LEDs;


	/*
	 * 	@priv_nh: 		used for grabbing params for launch files configs
	 * 	@camera_nh:		passed into the two seperate camera_info_managers that will be
	 *					created
	 * 	@single_camera_nh:
	 *					nodehandle for each seperate camera
	 *	@camera_name:	Used for Debug messages
	 *	@camera_frame:	Roslaunch parameter that allows user to say what the camera frame
	 *					is called
	 */
	ros::NodeHandle _priv_nh;
	ros::NodeHandle _camera_nh;
	ros::NodeHandle _single_camera_nh[TWO_CAMERAS];
	std::string		_camera_name; 	// = "duo3d_camera";
	std::string 	_camera_frame;


	/*
	* 	@brief 
	*	Fills the ros image msg's with the DUO frame buffer data
	*/
	void fillDUOImages(		sensor_msgs::Image& leftImage, 
							sensor_msgs::Image& rightImage, 
							const PDUOFrame pFrameData);

	/*
 	 * 	@brief 
 	 * 	Checking if the camerainfo we received from camera_info_manager is the same as the image
 	 * 	we are about to send.
	 *	
	 *	@WARN
	 *	Notify user if camera info and user specified camera settings are different, and that we
	 *	will still publish uncalibrated images.
	 */
	bool validateCameraInfo(const sensor_msgs::Image &image, const sensor_msgs::CameraInfo &ci);

	/*
	 *	@brief
	 * 	Pass ImagePtr array of size 2, containing both the left and right camera
	 * 	images that we got from the DUOCallback function to the fillDUOImages
	 *	function.
	 */
	void publishImages(		const sensor_msgs::ImagePtr image[TWO_CAMERAS]);

	/*
	 * 	@brief
	 * 	Two instances of image transport publishers; Left and Right publishers
	 */
	boost::shared_ptr<image_transport::ImageTransport> 	_it;
	image_transport::CameraPublisher 					_imagePub[TWO_CAMERAS];      

	/*
	 * 	@brief
	 * 	Create instance of CameraInfoManager for taking care of setting/getting
	 * 	calibration related information for camera info message.
	 */
	boost::shared_ptr<camera_info_manager::CameraInfoManager> _cinfo[TWO_CAMERAS];


	/* 
	 * 	@brief
	 * 	Check if the camera info matches the duo camera settings. If the camera settings
	 * 	change, then a warning will pop up saying you must recalibrate the stereo camera. 
	 */
	bool _calibrationMatches[TWO_CAMERAS];	 

	// 	Create transform broadcaster to transform image into the camera mount frame? 
	// 	Since the z axis is pointing into the image, but in the world z axis is up

	dynamic_reconfigure::Server<duo3d_ros::DuoConfig> 				_dynamicServer;
	dynamic_reconfigure::Server<duo3d_ros::DuoConfig>::CallbackType _serverCbType;


	void dynamicCallback(duo3d_ros::DuoConfig &config, uint32_t level);


	static DUOStereoDriver* pSingleton;
};

}

#endif
