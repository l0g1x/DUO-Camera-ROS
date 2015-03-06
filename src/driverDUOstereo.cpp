////////////////////////////////////////////////////////////////////
//
//	A ROS driver for the DUO3D Camera.
//	Author: Krystian Gebis
//	File:	driverDUOstereo.cpp
//
////////////////////////////////////////////////////////////////////

#include "driverDUOstereo.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

namespace duoStereo_driver
{

DUOStereoDriver* DUOStereoDriver::pSingleton(0L);

const std::string DUOStereoDriver::CameraNames[TWO_CAMERAS] = {"left","right"};


DUOStereoDriver::DUOStereoDriver(void):
	 _useDUO_Imu(false),
	_useDUO_LEDs(false),
	    _priv_nh("~"),
	  _camera_nh("duo3d_camera"),
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


DUOStereoDriver::~DUOStereoDriver(void)
{

}


void DUOStereoDriver::fillDUOImages(sensor_msgs::Image& leftImage, sensor_msgs::Image& rightImage, const PDUOFrame pFrameData)
{
	ros::Time timeNow 			= ros::Time::now();

	leftImage.header.stamp 		= timeNow;
	leftImage.header.frame_id 	= _camera_frame;
	rightImage.header.stamp 	= timeNow;
	rightImage.header.frame_id 	= _camera_frame;

	// Fill the left image message, the step size needs to be the amount of pixels
	// for the width of the image. fillImage() then allocates space in memory by making a
	// size_t = step * height    , where height is the amount of columns in pixels
	//
	sensor_msgs::fillImage(	leftImage, 								// image reference
							sensor_msgs::image_encodings::MONO8, 	// type of encoding
							pFrameData->height, 					// columns in pixels 
							pFrameData->width,						// rows in pixels
							pFrameData->width,						// step size 
							pFrameData->leftData);					// left camera data pointer

    sensor_msgs::fillImage( rightImage,
			                sensor_msgs::image_encodings::MONO8,
			                pFrameData->height,
			                pFrameData->width,
			                pFrameData->width,
			                pFrameData->rightData);
}


bool DUOStereoDriver::validateCameraInfo(const sensor_msgs::Image &image, const sensor_msgs::CameraInfo &ci)
{
	return (ci.width == image.width && ci.height == image.height);
}


void DUOStereoDriver::publishImages(const sensor_msgs::ImagePtr image[TWO_CAMERAS])
{

    for (int i = 0; i < TWO_CAMERAS; i++)
    {
        // Get current CameraInfo data and populate ImagePtr Array
        sensor_msgs::CameraInfoPtr
          		ci(new sensor_msgs::CameraInfo(_cinfo[i]->getCameraInfo()));

        // If camera info and image width and height dont match
        // then set calibration_matches to false so we then 
        // know if we should reset the camera info or not
        if (!validateCameraInfo(*image[i], *ci)) 
        {
        	if(_calibrationMatches[i])
        	{
        		_calibrationMatches[i] = false; 
        		ROS_WARN_STREAM("*" << _camera_name << "/" << CameraNames[i] << "* "
        							<< "camera_info is different then calibration info. "
        							<< "Uncalibrated image being sent.");
        	}
        	ci.reset(new sensor_msgs::CameraInfo());
            ci->height 	= image[i]->height;
            ci->width 	= image[i]->width;
        }
        else if (!_calibrationMatches[i])
        {
        	// Calibration is now okay
        	_calibrationMatches[i] = true; 
    		ROS_WARN_STREAM("*" << _camera_name << "/" << CameraNames[i] << "* "
    							<< "is now publishing calibrated images. ");
        }

        ci->header.frame_id = image[i]->header.frame_id;
        ci->header.stamp 	= image[i]->header.stamp;

        _imagePub[i].publish(image[i], ci);
    }

    sensor_msgs::clearImage(*image[0]);
	sensor_msgs::clearImage(*image[1]);
}


void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{

	// Using singleton to access DUOStereoDriver 
	// class member functions in this DUO C function
	DUOStereoDriver& 		duoDriver 	= DUOStereoDriver::GetInstance(); 	

	// Array to store left and right images
	sensor_msgs::ImagePtr 	image[duoDriver.TWO_CAMERAS];	

	// Initialize array of image pointers
    for (int i = 0; i < duoDriver.TWO_CAMERAS; i++)
    {
    	image[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);    	
    }

    // Dereferencing individual images to fill with pFrameData from camera
    // Then publish the images
    //duoDriver.fillDUOImages(*image[duoDriver.LEFT_CAM], *image[duoDriver.RIGHT_CAM], pFrameData);
    duoDriver.fillDUOImages(*image[1], *image[0], pFrameData);
    duoDriver.publishImages(image);

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
	 * We will use this to populate the image's message header.
	 */
	_priv_nh.param<std::string>("frame_id", _camera_frame, "duo3d_camera");


	/*
	 * @brief 
	 * @note
	 * NodeHandle param function does not use <Float> for input source (framesPerSecond) 
	 * so we have to pass as double, and then cast to float to satisfy DUOResolutionInfo 
	 * fps parameter requirement of type float
	 */
	int 	framesPerSecond;
	_priv_nh.param("FPS", framesPerSecond, 30);


	/* 
	 * Grab the resolution width and height, for temporary resolution enumeration
	 */
	int 	resWidth;
	int 	resHeight;
	_priv_nh.param("resolution_width", 	resWidth, 752);
	_priv_nh.param("resolution_height", resHeight, 480);


	// Find optimal binning parameters for given (width, height)
    // This maximizes sensor imaging area for given resolution
    int binning = DUO_BIN_NONE;
    if(resWidth <= 752/2) 
        binning += DUO_BIN_HORIZONTAL2;
    if(resHeight <= 480/4) 
        binning += DUO_BIN_VERTICAL4;
    else if(resHeight <= 480/2) 
        binning += DUO_BIN_VERTICAL2;

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



	// set camera_info_url using launch file
	std::string camera_info_url_left, camera_info_url_right;
	_priv_nh.param<std::string>("camera_info_left", camera_info_url_left, "");
	_priv_nh.param<std::string>("camera_info_right", camera_info_url_right, "");

	if( _cinfo[0]->validateURL( camera_info_url_left ) && _cinfo[1]->validateURL( camera_info_url_right ) )
	{
		_cinfo[0]->loadCameraInfo( camera_info_url_left );
		_cinfo[1]->loadCameraInfo( camera_info_url_right );

		ROS_INFO("custom DUO calibration files loaded");
	}
	else
	{
		ROS_ERROR("Calibration URL is invalid.");
	}


	/*
	 * @brief
	 * Select 752x480 resolution with no binning capturing at 30FPS
	 * These values (width, height, FPS) should be ROS Params
	 */
	if(EnumerateResolutions(&_duoResolutionInfo, 1, resWidth, resHeight, binning, framesPerSecond))
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
			SetDUOCameraSwap(_duoInstance, 1); // Switches left and right images

		}
		else
		{
			ROS_ERROR("Cannot Open DUO. Please check connection!");
			return false;
		}

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
// this callback function is called whenever the dynamic_reconfigure server (this node)
// recieves any new parameters to change. It basically changes the variables in this node
// based on the dynamic_reconfigure parameters it recieves.
void DUOStereoDriver::dynamicReconfigureCallback(duo3d_ros::DUO3DConfig &config, uint32_t level) 
{
  	ROS_DEBUG("Reconfigure Request: %d %d %d", 
            	config.exposure, config.gain, 
            	config.led_lighting);

  	// if any parameters have changed, then let the DUO camera know and change them
  	if(exposureDUO != config.exposure)
    	SetDUOExposure(_duoInstance, exposureDUO);
  	if(gainDUO != config.gain)
    	SetDUOGain(_duoInstance, gainDUO);
  	if(ledLightingDUO != config.led_lighting)
    	SetDUOLedPWM(_duoInstance, ledLightingDUO);

  	// store these new parameter values
  	exposureDUO 	= config.exposure;
  	gainDUO 		= config.gain;
  	ledLightingDUO 	= config.led_lighting;

}

void DUOStereoDriver::setup(void)
{
	_f = boost::bind(&DUOStereoDriver::dynamicReconfigureCallback, this, _1, _2);
	_dyParamSrv.setCallback(_f);
}
*/
void DUOStereoDriver::startDUO()
{
	// If we could successfully open the DUO, then lets start it to finish
	// the initialization 
	ROS_INFO("Starting DUO...");
	StartDUO(_duoInstance, DUOCallback, NULL);
	ROS_INFO("DUO Started.");
}


void DUOStereoDriver::shutdownDUO()
{
	ROS_WARN("Shutting down DUO Camera.");
	StopDUO(_duoInstance);
	CloseDUO(_duoInstance);
}

} // end namespace duoStereo_driver
