#include <DUOLib.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>
#include <dynamic_reconfigure/server.h>
#include <duo3d_ros/DUO3DConfig.h>

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

int exposure;
int gain;
int led_lighting;
      
void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData);
void dynamicReconfigureCallback(duo3d_ros::DUO3DConfig &config, uint32_t level);
