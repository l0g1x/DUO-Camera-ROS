#include <DUOLib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

bool Initialize();
		
DUOInstance 		duoInstance;
DUOResolutionInfo 	duoResolutionInfo;

char 	duoDeviceName[260];
char 	duoDeviceSerialNumber[260];
char 	duoDeviceFirmwareVersion[260];
char 	duoDeviceFirmwareBuild[260];

bool 	useDUO_Imu;
bool	useDUO_LEDs;

//ros::Publisher  leftImagePub;
//ros::Publisher  rightImagePub;

image_transport::CameraPublisher leftImagePub;      
image_transport::CameraPublisher rightImagePub;         


//sensor_msgs::Image 	leftImage;
//sensor_msgs::Image	rightImage;

//boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData);

