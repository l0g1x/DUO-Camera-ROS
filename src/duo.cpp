////////////////////////////////////////////////////////////////////
//
//	A ROS driver for the DUO3D Camera.
//	Author: Krystian Gebis
//	File:	driverDUOstereo.cpp
//
////////////////////////////////////////////////////////////////////

#include "duo.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <cv_bridge/cv_bridge.h>

namespace duoStereo_driver
{

DUOStereoDriver* DUOStereoDriver::pSingleton(0L);

const std::string DUOStereoDriver::CameraNames[TWO_CAMERAS] = {"left","right"};


DUOStereoDriver::DUOStereoDriver(void):
	 _useDUO_Imu(true),
	_useDUO_LEDs(true),
	    _priv_nh("~"),
	  _camera_nh("duo3d_camera"),
	_camera_name("duo3d_camera"),
	_duoInitialized(false),
	_publishDepth(true),
	_publishDepthImage(false),
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

	// Build color lookup table for depth display
	colorLut = Mat(cv::Size(256, 1), CV_8UC3);
	for(int i = 0; i < 256; i++)
		colorLut.at<Vec3b>(i) = (i==0) ? Vec3b(0, 0, 0) : HSV2RGB(i/256.0f, 1, 1);
}


DUOStereoDriver::~DUOStereoDriver(void)
{

}

bool DUOStereoDriver::useImuData()
{
	return _useDUO_Imu;
}

bool DUOStereoDriver::useDepthData()
{
	return _publishDepth;
}


void DUOStereoDriver::fillDUOImages(sensor_msgs::Image& leftImage, sensor_msgs::Image& rightImage, const PDUOFrame pFrameData)
{

	leftImage.header.stamp 		= ros::Time( double(pFrameData->timeStamp) * 1.e-4);
	leftImage.header.frame_id 	= _camera_frame;
	rightImage.header.stamp 	= leftImage.header.stamp;
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

void DUOStereoDriver::fillIMUData(sensor_msgs::Imu& imuData, std_msgs::Float32& tempData,  const PDUOFrame pFrameData)
{

	imuData.header.stamp = ros::Time( double(pFrameData->timeStamp) * 1.e-4);
	imuData.header.frame_id = _camera_frame;

	imuData.orientation.x = pFrameData->magData[0];
	imuData.orientation.y = pFrameData->magData[1];
	imuData.orientation.z = pFrameData->magData[2];

	imuData.angular_velocity.x = pFrameData->gyroData[0];
	imuData.angular_velocity.y = pFrameData->gyroData[1];
	imuData.angular_velocity.z = pFrameData->gyroData[2];

	imuData.linear_acceleration.x = pFrameData->accelData[0];
	imuData.linear_acceleration.y = pFrameData->accelData[1];
	imuData.linear_acceleration.z = pFrameData->accelData[2];

	tempData.data = pFrameData->tempData;
}

void DUOStereoDriver::fillDepthData(Mat3f depthMat, Mat1f dispMat, sensor_msgs::PointCloud2Ptr depthData, const PDUOFrame pFrameData)
{
	sensor_msgs::PointCloud2Modifier modifier(*depthData);
	/*modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
	                                            "y", 1, sensor_msgs::PointField::FLOAT32,
	                                            "z", 1, sensor_msgs::PointField::FLOAT32);
*/
	modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");


	uint32_t disparities;
	GetDense3DNumDisparities(_dense3dInstance, &disparities);
	Mat disp8;
	dispMat.convertTo(disp8, CV_8UC1, 255.0/(disparities*16));
	Mat mRGBDepth;
	cvtColor(disp8, mRGBDepth, COLOR_GRAY2BGR);
	LUT(mRGBDepth, colorLut, mRGBDepth);

	if (_publishDepthImage)
	{
		sensor_msgs::ImagePtr depthMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mRGBDepth).toImageMsg();
		_depthImagePub.publish(depthMsg);
	}

	modifier.resize(resWidth * resHeight);
	depthData->header.stamp = ros::Time( double(pFrameData->timeStamp) * 1.e-4);
	depthData->header.frame_id = _camera_frame;
	depthData->height = depthMat.rows;
	depthData->width = depthMat.cols;
	depthData->is_dense = false;
	depthData->is_bigendian = false;
	//depthData.point_step = 3;

	sensor_msgs::PointCloud2Iterator<float> iter_x(*depthData, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*depthData, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*depthData, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*depthData, "r");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*depthData, "g");
	sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*depthData, "b");

	float bad_point = std::numeric_limits<float>::quiet_NaN ();

	for (int v = 0; v < depthMat.rows; ++v)
	{
		for (int u = 0; u < depthMat.cols; ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
		{
			// x,y,z
			*iter_x = depthMat(v, u)[0] / 1000;
			*iter_y = depthMat(v, u)[1] / 1000;
			*iter_z = depthMat(v, u)[2] / 1000;

			cv::Vec3b& rgb = mRGBDepth.at<cv::Vec3b>(v, u);
			*iter_r = rgb[2];
			*iter_g = rgb[1];
			*iter_b = rgb[0];
		}
	}
}

void DUOStereoDriver::publishIMUData(const sensor_msgs::ImuPtr imuData, const std_msgs::Float32Ptr tempData) {
	tempPub.publish(tempData);
	imuPub.publish(imuData);
}

void DUOStereoDriver::publishDepthData(const sensor_msgs::PointCloud2Ptr depthData) {
	depthPub.publish(depthData);


	tf::Transform transform;
	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, depthData->header.stamp, "map", _camera_name));
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

    if (duoDriver.useImuData())
    {
		sensor_msgs::ImuPtr imu(new sensor_msgs::Imu);
		std_msgs::Float32Ptr temp(new std_msgs::Float32);
		duoDriver.fillIMUData(*imu, *temp, pFrameData);
		duoDriver.publishIMUData(imu, temp);

    }

    if (duoDriver.useDepthData())
    {
    	sensor_msgs::PointCloud2Ptr depthData(new sensor_msgs::PointCloud2);

    	Mat1f dispMat = Mat(Size(duoDriver.resWidth, duoDriver.resHeight), CV_32FC1);
    	Mat3f depthMat = duoDriver.getDepthData(pFrameData, dispMat);

    	duoDriver.fillDepthData(depthMat, dispMat, depthData, pFrameData);
    	duoDriver.publishDepthData(depthData);
    }

    duoDriver.publishImages(image);

}

Mat3f DUOStereoDriver::getDepthData(const PDUOFrame pFrameData, Mat1f disparity) {
	Mat3f depth3d = Mat(Size(resWidth, resHeight), CV_32FC3);
	PDense3DDepth data;// = new PDense3DDepth[resWidth * resHeight];
	if (!Dense3DGetDepth(_dense3dInstance, pFrameData->leftData, pFrameData->rightData,
							  (float*)disparity.data, (PDense3DDepth)depth3d.data)){
		Dense3DErrorCode error = Dense3DGetErrorCode();
		ROS_INFO("Could not get depth data. %i", (int) error);
	}

	return depth3d;
}

bool DUOStereoDriver::setCameraInfo(void)
{
	sensor_msgs::CameraInfo left_camera_info;
	sensor_msgs::CameraInfo right_camera_info;

	left_camera_info.width = _duoIntrinsics.w;
	left_camera_info.height = _duoIntrinsics.h;
	right_camera_info.width = _duoIntrinsics.w;
	right_camera_info.height = _duoIntrinsics.h;

	for(int i = 0; i < 9; i++)
	{
		left_camera_info.R[i] = _duoExtrinsics.rotation[i];
		right_camera_info.R[i] = _duoExtrinsics.rotation[i];
	}

	for(int i = 0; i < 12; i++)
	{
		left_camera_info.P[i] = _duoIntrinsics.left[i];
		right_camera_info.P[i] = _duoIntrinsics.right[i];
	}

	_cinfo[0]->setCameraInfo(left_camera_info);
	_cinfo[1]->setCameraInfo(right_camera_info);
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
	double	framesPerSecond;
	_priv_nh.param("FPS", framesPerSecond, 30.0);


	/* 
	 * Grab the resolution width and height, for temporary resolution enumeration
	 */
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
	_priv_nh.param<bool>("publishDepth", _publishDepth, false);
	_priv_nh.param<bool>("publishDepthImage", _publishDepthImage, false);

	if (_useDUO_Imu)
	{
		tempPub = _camera_nh.advertise<std_msgs::Float32>("temperature", 10);
		imuPub = _camera_nh.advertise<sensor_msgs::Imu>("imu", 10);
	}

	if (_publishDepth)
	{
		depthPub = _camera_nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 2);
	}

	if (_publishDepthImage)
	{
		_depthImagePub = _it->advertise("depth_image", 1);
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


			_priv_nh.param("exposure"		, _duoExposure	, 50.0);
			_priv_nh.param("gain"			, _duoGain 		, 50.0);
			_priv_nh.param("led_lighting"	, _duoLEDLevel	, 50.0);

			// These need to be roslaunch parameters. Will make dynamic reconfig 
			SetDUOExposure(_duoInstance, _duoExposure);
			SetDUOGain(_duoInstance, _duoGain);
			SetDUOLedPWM(_duoInstance, _duoLEDLevel);
			SetDUOCameraSwap(_duoInstance, _duoCameraSwap); // Switches left and right images
			SetDUOUndistort(_duoInstance, true);

			// Get DUO calibration intrinsics and extrinsics
			if(!GetDUOCalibrationPresent(_duoInstance))
			{
				ROS_ERROR("DUO camera calibration data not present\n");
				return false;
			}
			else
			{
				bool result = GetDUOIntrinsics(_duoInstance, &_duoIntrinsics);

				if (result)
					result = GetDUOExtrinsics(_duoInstance, &_duoExtrinsics);

				if (!result)
				{
					ROS_ERROR("Could not get DUO camera calibration data\n");
					return false;
				}
			}

			setCameraInfo();

		}
		else
		{
			ROS_ERROR("Cannot Open DUO. Please check connection!");
			return false;
		}

		_duoInitialized = true;

		if (_publishDepth)
			initializeDense3D();
		return true;
	}
	else
	{
		ROS_ERROR("Resolution Parameters Check: FAILED");
		return false;
	}

	return false;
}

bool DUOStereoDriver::initializeDense3D() {

	_priv_nh.param("numDisparities", _numDisparities, 2);
	_priv_nh.param("sadWindowSize"		, _sadWindowSize 		, 6);
	_priv_nh.param("preFilterCap"		, _preFilterCap			, 28);
	_priv_nh.param("uniquenessRatio"	, _uniquenessRatio		, 27);
	_priv_nh.param("speckleWindowSize"	, _speckleWindowSize	, 52);
	_priv_nh.param("speckleRange"		, _speckleRange			, 14);
	_priv_nh.param("dense3dMode"		, _dense3dMode			, 0);

	ROS_INFO("Initiliazing DUO Dense 3D");
	if (!Dense3DOpen(&_dense3dInstance))
	{
		ROS_ERROR("Could not open dense 3d");
	}

	_priv_nh.param<std::string>("dense3dLicense"		, _duoDense3dLicense	, "96WWM-3C2W6-AZ66J-P69MJ-LBFHK");

	//if (!SetDense3DLicense(&_dense3dInstance, _duoDense3dLicense.c_str()))
	if (!SetDense3DLicense(_dense3dInstance, "96WWM-3C2W6-AZ66J-P69MJ-LBFHK"))
	{
		ROS_ERROR("Invalid license key for Dense3D");
	}


	SetDense3DMode(_dense3dInstance, _dense3dMode);
	SetDense3DNumDisparities(_dense3dInstance, _numDisparities);
	SetDense3DSADWindowSize(_dense3dInstance, _sadWindowSize);
	SetDense3DPreFilterCap(_dense3dInstance, _preFilterCap);
	SetDense3DUniquenessRatio(_dense3dInstance, _uniquenessRatio);
	SetDense3DSpeckleWindowSize(_dense3dInstance, _speckleWindowSize);
	SetDense3DSpeckleRange(_dense3dInstance, _speckleRange);

	if (!SetDense3DImageSize(_dense3dInstance, resWidth, resHeight))
	{
		ROS_ERROR("Could not get set Dense3D image size\n");
		return false;
	}

	// Set Dense3D parameters
	SetDense3DCalibration(_dense3dInstance, &_duoIntrinsics, &_duoExtrinsics);

	return true;
}

// this callback function is called whenever the dynamic_reconfigure server (this node)
// recieves any new parameters to change. It basically changes the variables in this node
// based on the dynamic_reconfigure parameters it recieves.
void DUOStereoDriver::dynamicCallback(duo3d_ros::DuoConfig &config, uint32_t level) 
{
  	//ROS_INFO("Reconfigure Request: %f %f %f", 
    //        	config.exposure, config.gain, 
    //        	config.LED);

  	// if any parameters have changed, then let the DUO camera know and change them
  	if(_duoExposure != config.exposure)
  	{
  		_duoExposure = config.exposure;
    	SetDUOExposure(_duoInstance, _duoExposure);
  	}
  	if(_duoGain != config.gain)
  	{
  		_duoGain = config.gain;
    	SetDUOGain(_duoInstance, _duoGain);
  	}
  	if(_duoLEDLevel != config.LED)
  	{
  		_duoLEDLevel = config.LED;
    	SetDUOLedPWM(_duoInstance, _duoLEDLevel);
  	}
  	if(_duoCameraSwap != config.CameraSwap)
  	{
  		_duoCameraSwap = config.CameraSwap;
  		SetDUOCameraSwap(_duoInstance, _duoCameraSwap);
  	}
  	if(_duoHorizontalFlip != config.HorizontalFlip)
  	{
  		_duoHorizontalFlip = config.HorizontalFlip;
  		SetDUOHFlip(_duoInstance, _duoHorizontalFlip);
  	}
  	if(_duoVerticalFlip != config.VerticalFlip)
  	{
  		_duoVerticalFlip = config.VerticalFlip;
  		SetDUOVFlip(_duoInstance, _duoVerticalFlip);
  	}

  	if (_numDisparities != config.numDisparities){
		_numDisparities = config.numDisparities;
		SetDense3DNumDisparities(_dense3dInstance, _numDisparities);
	}

	if (_sadWindowSize != config.sadWindowSize) {
		_sadWindowSize = config.sadWindowSize;
		SetDense3DSADWindowSize(_dense3dInstance, _sadWindowSize);
	}

	if (_preFilterCap != config.preFilterCap) {
		_preFilterCap = config.preFilterCap;
		SetDense3DPreFilterCap(_dense3dInstance, _preFilterCap);
	}

	if (_uniquenessRatio != config.uniquenessRatio) {
		_uniquenessRatio = config.uniquenessRatio;
		SetDense3DUniquenessRatio(_dense3dInstance, _uniquenessRatio);
	}

	if (_speckleWindowSize != config.speckleWindowSize) {
		_speckleWindowSize = config.speckleWindowSize;
		SetDense3DSpeckleWindowSize(_dense3dInstance, _speckleWindowSize);
	}

	if (_speckleRange != config.speckleRange) {
		_speckleRange = config.speckleRange;
		SetDense3DSpeckleRange(_dense3dInstance, _speckleRange);
	}

	if (_dense3dMode != config.dense3dMode) {
		_dense3dMode = config.dense3dMode;
		SetDense3DMode(_dense3dInstance, _dense3dMode);
	}

}


void DUOStereoDriver::setup(void)
{
	_serverCbType = boost::bind(&DUOStereoDriver::dynamicCallback, this, _1, _2);
	_dynamicServer.setCallback(_serverCbType);
}


void DUOStereoDriver::startDUO()
{
	// If we could successfully open the DUO, then lets start it to finish
	// the initialization 
	ROS_INFO("Starting DUO...");
	StartDUO(_duoInstance, DUOCallback, NULL);
	ROS_INFO("DUO Started.");

	if (_publishDepth)
		startDense3D();
}

void DUOStereoDriver::startDense3D()
{
	// If we could successfully open the DUO, then lets start it to finish
	// the initialization


}


void DUOStereoDriver::shutdownDUO()
{
	if (_publishDepth)
		shutdownDense3D();

	ROS_WARN("Shutting down DUO Camera.");
	StopDUO(_duoInstance);
	CloseDUO(_duoInstance);
}

void DUOStereoDriver::shutdownDense3D()
{
	ROS_WARN("Shutting down DUO Camera.");
	if (_dense3dInstance != NULL)
		Dense3DClose(&_dense3dInstance);
}

Vec3b DUOStereoDriver::HSV2RGB(float hue, float sat, float val)
{
	float x, y, z;

	if(hue == 1) hue = 0;
	else         hue *= 6;

	int i = static_cast<int>(floorf(hue));
	float f = hue - i;
	float p = val * (1 - sat);
	float q = val * (1 - (sat * f));
	float t = val * (1 - (sat * (1 - f)));

	switch(i)
	{
		case 0: x = val; y = t; z = p; break;
		case 1: x = q; y = val; z = p; break;
		case 2: x = p; y = val; z = t; break;
		case 3: x = p; y = q; z = val; break;
		case 4: x = t; y = p; z = val; break;
		case 5: x = val; y = p; z = q; break;
	}
	return Vec3b((uchar)(x * 255), (uchar)(y * 255), (uchar)(z * 255));
}

} // end namespace duoStereo_driver
