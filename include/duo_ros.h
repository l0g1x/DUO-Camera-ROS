#include <DUOLib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

bool Initialize();
		
DUOInstance 		duoInstance;
DUOResolutionInfo 	duoResolutionInfo;

char 	duoDeviceName[260];
char 	duoDeviceSerialNumber[260];
char 	duoDeviceFirmwareVersion[260];
char 	duoDeviceFirmwareBuild[260];

bool 	useDUO_Imu;
bool	useDUO_LEDs;

ros::Publisher  leftImagePub;
ros::Publisher  rightImagePub;

sensor_msgs::Image 	leftImage;
sensor_msgs::Image	rightImage;

void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData);

