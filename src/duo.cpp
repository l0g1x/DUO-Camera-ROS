#include "duo_ros.h"


bool Initialize()
{
	// Implement libCheck() later to tell user they need to update their DUO SDK
	ROS_INFO("DUOLib Version: -----> v%s <-----\n", GetLibVersion());

	// Creating local nodehandler to access private parameters set in the 
	// launch file using <param> tag
	//
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

	// NodeHandle param function does not use <Float> for input source (framesPerSecond) 
	// so we have to pass as double, and then cast to float to satisfy DUOResolutionInfo 
	// fps parameter requirement of type float
	//
	double 	framesPerSecond;
	nLocal.param("FPS", framesPerSecond, 30.0);

	// Grab the resolution width and height, for temporary resolution enumeration
	//
	int 	resWidth;
	int 	resHeight;
	nLocal.param("resolution_width", 	resWidth, 320);
	nLocal.param("resolution_height", 	resHeight, 240);

	// Grab bool for whether user wants to use IMU &/or LED's
	//
	nLocal.param<bool>("use_DUO_imu",  useDUO_Imu,  false);
	nLocal.param<bool>("use_DUO_LEDs", useDUO_LEDs, false);



    	// Select 320x240 resolution with 2x2 binning capturing at 10FPS
	// These values should be ROS Params
	// As of right now, no binning since I dont know anything about binning (yet...)
	//
	if(EnumerateResolutions(&duoResolutionInfo, 1, 752, 480, DUO_BIN_NONE, 20))
	{
		if(OpenDUO(&duoInstance))
		{
			GetDUODeviceName(	duoInstance, duoDeviceName);
			GetDUOSerialNumber(	duoInstance, duoDeviceSerialNumber);
			GetDUOFirmwareVersion(	duoInstance, duoDeviceFirmwareVersion);
			GetDUOFirmwareBuild(	duoInstance, duoDeviceFirmwareBuild);
			SetDUOResolutionInfo( 	duoInstance, duoResolutionInfo);

			int exposure;
			int gain;
			int led_lighting;
			nLocal.param("exposure"		, exposure	, 50);
			nLocal.param("gain"		, gain 		, 50);
			nLocal.param("led_lighting"	, led_lighting	, 50);

			// These need to be roslaunch parameters. Will make dynamic reconfig 
			SetDUOExposure(duoInstance, exposure);
			SetDUOGain(duoInstance, gain);
			SetDUOLedPWM(duoInstance, led_lighting);

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

ros::NodeHandle* pcamera_nh;

#include <cassert>

void publishImages(sensor_msgs::Image &leftImage, sensor_msgs::Image &rightImage)
{
	boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo( new camera_info_manager::CameraInfoManager(*pcamera_nh));
	cinfo->setCameraName("duo3d_camera");
	sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo->getCameraInfo()));
	
	ci.reset(new sensor_msgs::CameraInfo());
	ci->height = leftImage.height;
	ci->width  = leftImage.width;

	ci->header.frame_id = leftImage.header.frame_id;
    ci->header.stamp = leftImage.header.stamp;

    leftImagePub.publish(leftImage, *ci);
    rightImagePub.publish(rightImage, *ci);
}


// DUOFrameCallback that gets fired everytime the camera sends a new image
// Right now it is not using the polling mechanism, which it eventually should be using 
// since we do not want to spend much time in this callback, as said in the DUO documentation
//
void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
	ros::Time 	timeNow		= ros::Time::now();
	std::string	frame		= "duo3d_camera";

	sensor_msgs::Image leftImage;
	sensor_msgs::Image rightImage;

	// leftImage/rightImage defined in header
	leftImage.header.stamp 		= timeNow;
	leftImage.header.frame_id 	= frame;

	rightImage.header.stamp		= timeNow;
	rightImage.header.frame_id	= frame;

	// Fill the left image message, the step size needs to be the amount of pixels
	// for the width of the image. fillImage() then allocates space in memory by making a
	// size_t = step * height    , where height is the amount of columns in pixels
	//
	sensor_msgs::fillImage(	leftImage, 
				sensor_msgs::image_encodings::MONO8, 
				pFrameData->height, 	// columns in pixels 
				pFrameData->width,	// rows in pixels
				pFrameData->width,	// step size 
				pFrameData->leftData);	// left camera data pointer

        sensor_msgs::fillImage( rightImage,
                                sensor_msgs::image_encodings::MONO8,
                                pFrameData->height,
                                pFrameData->width,
                                pFrameData->width,
                                pFrameData->rightData);

	// Publish the sensor_msgs::Image message
	//
	publishImages(leftImage, rightImage);

	// Clear the image after we sent it. It is part of the fill_image.h
	//
	sensor_msgs::clearImage(leftImage);
	sensor_msgs::clearImage(rightImage);

	
    	ROS_DEBUG("  Timestamp:          %10.1f ms\n", pFrameData->timeStamp/10.0f);
    	ROS_DEBUG("  Frame Size:         %dx%d\n", pFrameData->width, pFrameData->height);
    	ROS_DEBUG("  Left Frame Buffer:  %p\n", pFrameData->leftData);
    	ROS_DEBUG("  Right Frame Buffer: %p\n", pFrameData->rightData);
    	ROS_DEBUG("------------------------------------------------------\n");
}

int main(int argc, char** argv)
{
	// Initializing the ros node. Must call ros::init() before any NodeHandle
	// constructor is called
	//
    	ros::init(argc, argv, "duo3d_camera");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");
	ros::NodeHandle camera_nh("duo3d_camera");

	pcamera_nh = &camera_nh;
	boost::shared_ptr<image_transport::ImageTransport> it(new image_transport::ImageTransport(camera_nh));

	leftImagePub   = it->advertiseCamera("left/image_raw", 1);	
 	rightImagePub  = it->advertiseCamera("right/image_raw", 1);	

	ros::Rate loop_rate(30);

	// initialize the left/right image publishers
	//
	//leftImagePub 	= n.advertise<sensor_msgs::Image>("image_raw/left", 1);
	//rightImagePub 	= n.advertise<sensor_msgs::Image>("image_raw/right", 1);

	// If we successfully initialized, then start the duo, and loop while
	// the ros node is ok (did not fail, or someone ctrl+c)
	//
	if (Initialize() == true)
	{
		StartDUO(duoInstance, DUOCallback, NULL);
		while(ros::ok())
		{
			ros::spinOnce();
			loop_rate.sleep();
		}
		StopDUO(duoInstance);
		CloseDUO(duoInstance);		
	}
	else
	{
		ROS_ERROR("Initialization failed :(");
	}

	return 0;
}
