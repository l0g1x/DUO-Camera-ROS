#include "duo_ros.h"


bool Initialize()
{
	ROS_INFO("DUOLib Version: -----> v%s <-----\n", GetLibVersion());

	ros::NodeHandle nLocal("~");

	std::string 	deviceName;
	if(nLocal.getParam("device_name", deviceName))
	{
		ROS_INFO_STREAM("DUO Device: " << deviceName); 
	}
	else
	{
		ROS_FATAL("No Device Name! Please set the 'device_name' parameter.");
	}

	std::string 	deviceSerialNum;
	if(nLocal.getParam("device_serial_number", deviceSerialNum))
	{
		// if(isValidSerialNumber(deviceSerialNum))
		if(deviceSerialNum != "foo")
		{
			ROS_INFO("Device Serial Check: PASSED");
		}
		else
		{
			ROS_FATAL("Device Serial Check: FAILED");
		}
	}
	else
	{
		ROS_FATAL("No Serial Number! Please set the 'device_serial_number' parameter.");
	}

	std::string 	cameraFrame;
	nLocal.param<std::string>("frame_id", cameraFrame, "duo3d_camera");

	// @brief: 	param function does not use <Float> for input source (framesPerSecond) 
			so we have to pass double, and then cast to float to satisfy DUOResolutionInfo 
			fps variable of type float
	double 	framesPerSecond;
	nLocal.param("FPS", framesPerSecond, 30.0);

	int 	resWidth;
	int 	resHeight;
	nLocal.param("resolution_width", 	resWidth, 320);
	nLocal.param("resolution_height", 	resHeight, 240);

	nLocal.param<bool>("use_DUO_imu",  useDUO_Imu,  false);
	nLocal.param<bool>("use_DUO_LEDs", useDUO_LEDs, false);



    	// Select 320x240 resolution with 2x2 binning capturing at 10FPS
	// These values should be ROS Params
	if(EnumerateResolutions(&duoResolutionInfo, 1, 320, 240, DUO_BIN_HORIZONTAL2+DUO_BIN_VERTICAL2, 30))
	{
		if(OpenDUO(&duoInstance))
		{
			GetDUODeviceName(	duoInstance, duoDeviceName);
			GetDUOSerialNumber(	duoInstance, duoDeviceSerialNumber);
			GetDUOFirmwareVersion(	duoInstance, duoDeviceFirmwareVersion);
			GetDUOFirmwareBuild(	duoInstance, duoDeviceFirmwareBuild);
			SetDUOResolutionInfo( 	duoInstance, duoResolutionInfo);

			return true;
		}
		else
		{
			ROS_FATAL("Cannot Open DUO. Please check connection!");
			return false;
		}
	}

	return false;
}

void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
    ROS_INFO("  Timestamp:          %10.1f ms\n", pFrameData->timeStamp/10.0f);
    ROS_INFO("  Frame Size:         %dx%d\n", pFrameData->width, pFrameData->height);
    ROS_INFO("  Left Frame Buffer:  %p\n", pFrameData->leftData);
    ROS_INFO("  Right Frame Buffer: %p\n", pFrameData->rightData);
    ROS_INFO("------------------------------------------------------\n");
}

int main(int argc, char** argv)
{
    	ros::init(argc, argv, "duo3d_camera");
	ros::NodeHandle n;

	if (Initialize() == true)
	{
		// Temporary
		StartDUO(duoInstance, DUOCallback, NULL);
		StopDUO(duoInstance);
		CloseDUO(duoInstance);		
	}
	else
	{
		ROS_ERROR("Initialization failed :(");
	}

	return 0;
}
