#include "driverDUOstereo.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

namespace duoStereo_driver
{

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

bool DUOStereoDriver::InitializeDUO()
{
	// Implement libCheck() later to tell user they need to update their DUO SDK
	ROS_INFO("DUOLib Version: -----> v%s <-----\n", GetLibVersion());

	// Creating local nodehandler to access private parameters set in the 
	// launch file using <param> tag
	//

	// std::string 	deviceName;
	// if(_priv_nh.getParam("device_name", deviceName))
	// {
	// 	ROS_INFO_STREAM("DUO Device: " << deviceName); 
	// }
	// else
	// {
	// 	ROS_FATAL("No Device Name! Please set the 'device_name' parameter.");
	// }


    // Select 752x480 resolution with no binning capturing at 20FPS
	// These values should be ROS Params
	//
	// if(EnumerateResolutions(&duoResolutionInfo, 1, 752, 480, DUO_BIN_NONE, 20))
	// {
	// 	if(OpenDUO(&duoInstance))
	// 	{
	// 		GetDUODeviceName(		duoInstance, duoDeviceName);
	// 		GetDUOSerialNumber(		duoInstance, duoDeviceSerialNumber);
	// 		GetDUOFirmwareVersion(	duoInstance, duoDeviceFirmwareVersion);
	// 		GetDUOFirmwareBuild(	duoInstance, duoDeviceFirmwareBuild);
	// 		SetDUOResolutionInfo( 	duoInstance, duoResolutionInfo);

	// 		int exposure;
	// 		int gain;
	// 		int led_lighting;
	// 		nLocal.param("exposure"		, exposure		, 50);
	// 		nLocal.param("gain"			, gain 			, 50);
	// 		nLocal.param("led_lighting"	, led_lighting	, 50);

	// 		// These need to be roslaunch parameters. Will make dynamic reconfig 
	// 		SetDUOExposure(duoInstance, exposure);
	// 		SetDUOGain(duoInstance, gain);
	// 		SetDUOLedPWM(duoInstance, led_lighting);

	// 		return true;
	// 	}
	// 	else
	// 	{
	// 		ROS_FATAL("Cannot Open DUO. Please check connection!");
	// 		return false;
	// 	}
	// }

	return false;
}

} // end namespace duoStereo_driver
