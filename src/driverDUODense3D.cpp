////////////////////////////////////////////////////////////////////
//
//	A ROS driver for the DUO3D Camera.
//	Author: Krystian Gebis
//	File:	driverDUOstereo.cpp
//
////////////////////////////////////////////////////////////////////

#include "driverDUODense3D.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

namespace duoDense3D_driver
{

DUODense3DDriver* DUODense3DDriver::pSingleton(0L);


DUODense3DDriver::DUODense3DDriver(void):
	    _priv_nh("~"),
		_speckleWindowSize(0),
		_speckleRange(0),
		_uniquenessRatio(0),
		_p1(800),
		_p2(1600),
		_preFilterCap(),
		_numDisparities(3),
		_sadWindowSize(2),
		_duoStereoDriver(DUOStereoDriver::GetInstance()){
}


DUODense3DDriver::~DUODense3DDriver(void)
{

}


bool DUODense3DDriver::initializeDense3D()
{
	// Implement libCheck() later to tell user they need to update their DUO SDK
	ROS_DEBUG("DUOLib Dense3D Version: %s", Dense3DGetLibVersion());
	ROS_INFO("DUOLib Dense3D Version: %s", Dense3DGetLibVersion());

	if (_duoStereoDriver.initializeDUO())
		return false;

	_priv_nh.param("numDisparities", _numDisparities, 3);
	_priv_nh.param("sadWindowSize"		, _sadWindowSize 		, 2);
	_priv_nh.param("p1"					, _p1					, 800);
	_priv_nh.param("p2"					, _p2					, 1600);
	_priv_nh.param("preFilterCap"		, _preFilterCap			, 0);
	_priv_nh.param("uniquenessRatio"	, _uniquenessRatio		, 0);
	_priv_nh.param("speckleWindowSize"	, _speckleWindowSize	, 0);
	_priv_nh.param("speckleRange"		, _speckleRange			, 0);


	SetDense3DNumDisparities(&_dense3dInstance, _numDisparities);
	SetDense3DSADWindowSize(&_dense3dInstance, _sadWindowSize);
	SetDense3DP1(&_dense3dInstance, _p1);
	SetDense3DP2(&_dense3dInstance, _p2);
	SetDense3DPreFilterCap(&_dense3dInstance, _preFilterCap);
	SetDense3DUniquenessRatio(&_dense3dInstance, _uniquenessRatio);
	SetDense3DSpeckleWindowSize(&_dense3dInstance, _speckleWindowSize);
	SetDense3DSpeckleRange(&_dense3dInstance, _speckleRange);

	return true;
}

// this callback function is called whenever the dynamic_reconfigure server (this node)
// recieves any new parameters to change. It basically changes the variables in this node
// based on the dynamic_reconfigure parameters it recieves.
void DUODense3DDriver::dynamicCallback(duo3d_ros::Dense3DConfig &config, uint32_t level)
{
  	//ROS_INFO("Reconfigure Request: %f %f %f",
    //        	config.exposure, config.gain,
    //        	config.LED);
	if (_numDisparities != config.numDisparities){
		_numDisparities = config.numDisparities;
		SetDense3DNumDisparities(&_dense3dInstance, _numDisparities);
	}

	if (_sadWindowSize != config.sadWindowSize) {
		_sadWindowSize = config.sadWindowSize;
		SetDense3DSADWindowSize(&_dense3dInstance, _sadWindowSize);
	}

	if (_p1 != config.p1) {
		_p1 = config.p1;
		SetDense3DP1(&_dense3dInstance, _p1);
	}

	if (_p2 != config.p2) {
		_p2 = config.p2;
		SetDense3DP2(&_dense3dInstance, _p2);
	}

	if (_preFilterCap != config.preFilterCap) {
		_preFilterCap = config.preFilterCap;
		SetDense3DPreFilterCap(&_dense3dInstance, _preFilterCap);
	}

	if (_uniquenessRatio != config.uniquenessRatio) {
		_uniquenessRatio = config.uniquenessRatio;
		SetDense3DUniquenessRatio(&_dense3dInstance, _uniquenessRatio);
	}

	if (_speckleWindowSize != config.speckleWindowSize) {
		_speckleWindowSize = config.speckleWindowSize;
		SetDense3DSpeckleWindowSize(&_dense3dInstance, _speckleWindowSize);
	}

	if (_speckleRange != config.speckleRange) {
		_speckleRange = config.speckleRange;
		SetDense3DSpeckleRange(&_dense3dInstance, _speckleRange);
	}


}

void DUODense3DDriver::setParameters() {
	//SetDense3DCalibration(&_dense3dInstance, &intr, &extr);


}

void DUODense3DDriver::setup(void)
{
	_serverCbType = boost::bind(&DUODense3DDriver::dynamicCallback, this, _1, _2);
	_dynamicServer.setCallback(_serverCbType);
}


void DUODense3DDriver::startDense3D()
{
	// If we could successfully open the DUO, then lets start it to finish
	// the initialization
	ROS_INFO("Starting DUO...");
	_duoStereoDriver.setup();
	_duoStereoDriver.startDUO();
	ROS_INFO("DUO Started.");

	ROS_INFO("Starting DUO Dense 3D");
	if (!Dense3DOpen(&_dense3dInstance))
	{
		ROS_ERROR("Could not open dense 3d");
	}

	if (!SetDense3DLicense(&_dense3dInstance, "96WWM-3C2W6-AZ66J-P69MJ-LBFHK"))
	{
		ROS_ERROR("Invalid license key for Dense3D");
	}

}


void DUODense3DDriver::shutdownDense3D()
{
	ROS_WARN("Shutting down DUO Camera.");
	if (_dense3dInstance != NULL)
		Dense3DClose(&_dense3dInstance);
}

} // end namespace duoDense3D_driver
