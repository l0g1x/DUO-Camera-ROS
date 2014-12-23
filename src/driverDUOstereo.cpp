#include "driverDUOstereo.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

namespace duoStereo_driver
{

DUOStereoDriver* DUOStereoDriver::pSingleton(0L);

const std::string DUOStereoDriver::CameraNames[TWO_CAMERAS] = {"left","right"};

DUOStereoDriver::DUOStereoDriver(ros::NodeHandle priv_nh, 
								ros::NodeHandle camera_nh):
	 _useDUO_Imu(false),
	_useDUO_LEDs(false),
	    _priv_nh(priv_nh),
	  _camera_nh(camera_nh),
	_camera_name("duo_camera"),
	         _it(new image_transport::ImageTransport(_camera_nh))
{
	for(int i = 0; i < TWO_CAMERAS; i++)
	{
		_single_camera_nh[i] 	= ros::NodeHandle(_camera_nh, CameraNames[i]);  // for i-th CameraInfoManager
      	_cinfo[i] 				= boost::shared_ptr<camera_info_manager::CameraInfoManager>
      								(new camera_info_manager::CameraInfoManager(_single_camera_nh[i]));
      	_calibrationMatches[i] 	= true;
      	_imagePub[i] 			= _it->advertiseCamera(CameraNames[i]+"/image_raw", 1);
	}
}

DUOStereoDriver::~DUOStereoDriver()
{

}

void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
	DUOStereoDriver& duoDriver = DUOStereoDriver::GetInstance();
	// sensor_msgs::ImagePtr image[TWO_CAMERAS];

 //    for (int i = 0; i<TWO_CAMERAS; i++)
 //    {
 //    	image[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);    	
 //    }

}

bool DUOStereoDriver::initializeDUO()
{
	// Implement libCheck() later to tell user they need to update their DUO SDK
	ROS_DEBUG("DUOLib Version: %s", GetLibVersion());


	std::string 	deviceName;
	if(_priv_nh.getParam("device_name", deviceName))
	{
		ROS_INFO_STREAM("DUO Device: " << deviceName); 
	}
	else
	{
		ROS_ERROR("No Device Name! Please set the 'device_name' parameter.");
		return false;
	}


	std::string 	deviceSerialNum;
	if(_priv_nh.getParam("device_serial_number", deviceSerialNum))
	{
		// if(isValidSerialNumber(deviceSerialNum))
		if(deviceSerialNum != "foo")
		{
			ROS_INFO("Device Serial Check: PASSED");
		}
		else
		{
			ROS_ERROR("Device Serial Check: FAILED");
			return false;
		}
	}
	else
	{
		ROS_ERROR("No Serial Number! Please set the 'device_serial_number' parameter.");
		return false;
	}

	/*
	 * We will use this to populate the image's message header
	 */
	std::string 	cameraFrame;
	_priv_nh.param<std::string>("frame_id", cameraFrame, "duo3d_camera");


	/*
	 * @brief 
	 * @note
	 * NodeHandle param function does not use <Float> for input source (framesPerSecond) 
	 * so we have to pass as double, and then cast to float to satisfy DUOResolutionInfo 
	 * fps parameter requirement of type float
	 */
	double 	framesPerSecond;
	_priv_nh.param("FPS", framesPerSecond, 30.0);


	/* 
	 * Grab the resolution width and height, for temporary resolution enumeration
	 */
	int 	resWidth;
	int 	resHeight;
	_priv_nh.param("resolution_width", 	resWidth, 752);
	_priv_nh.param("resolution_height", resHeight, 480);


	/*
	 * @brief
	 * Grab bool for whether user wants to use IMU &/or LED's
	 *
	 * @TODO
	 * Change this to local variable within this initializeDUO() 
	 * function and tell the DUO that we want the imu data as well.
	 * If it already being sent without even trying to turn it on,
	 * figure out how it can be turned off so that we use less 
	 * resources.
	 */
	_priv_nh.param<bool>("use_DUO_imu",  _useDUO_Imu,  false);
	_priv_nh.param<bool>("use_DUO_LEDs", _useDUO_LEDs, false);


	/*
	 * @brief
	 * Select 752x480 resolution with no binning capturing at 20FPS
	 * These values (width, height, FPS) should be ROS Params
	 */
	if(EnumerateResolutions(&_duoResolutionInfo, 1, resWidth, resHeight, DUO_BIN_NONE, 20))
	{
		ROS_INFO("Resolution Parameters Check: PASSED");
		if(OpenDUO(&_duoInstance))
		{
			GetDUODeviceName(		_duoInstance, _duoDeviceName);
			GetDUOSerialNumber(		_duoInstance, _duoDeviceSerialNumber);
			GetDUOFirmwareVersion(	_duoInstance, _duoDeviceFirmwareVersion);
			GetDUOFirmwareBuild(	_duoInstance, _duoDeviceFirmwareBuild);
			SetDUOResolutionInfo( 	_duoInstance, _duoResolutionInfo);

			int exposure;
			int gain;
			int led_lighting;
			_priv_nh.param("exposure"		, exposure		, 50);
			_priv_nh.param("gain"			, gain 			, 50);
			_priv_nh.param("led_lighting"	, led_lighting	, 50);

			// These need to be roslaunch parameters. Will make dynamic reconfig 
			SetDUOExposure(_duoInstance, exposure);
			SetDUOGain(_duoInstance, gain);
			SetDUOLedPWM(_duoInstance, led_lighting);

		}
		else
		{
			ROS_ERROR("Cannot Open DUO. Please check connection!");
			return false;
		}

		// If we could successfully open the DUO, then lets start it to finish
		// the initialization 
		ROS_INFO("Starting DUO...");
		StartDUO(_duoInstance, DUOCallback, NULL);
		ROS_INFO("DUO Started.");
		return true;
	}
	else
	{
		ROS_ERROR("Resolution Parameters Check: FAILED");
		return false;
	}

	return false;
}


/*
 * @brief
 * Using the DUO API function calls to properly end connection
 * with DUO camera. This should ONLY be called, if the ros node 
 * receives a shutdown signal, or the node.ok() returns false.
 */
void DUOStereoDriver::shutdownDUO()
{
	ROS_DEBUG("Shutting down DUO Camera.");
	StopDUO(_duoInstance);
	CloseDUO(_duoInstance);
}

} // end namespace duoStereo_driver
