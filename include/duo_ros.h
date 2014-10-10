#include <DUOLib.h>
#include <stdio.h>
#include <ros/ros.h>

class DUO
{
	public:
		void Initialize(void);
	private:
		ros::NodeHandle 	nh;
		
		DUOInstance 		duoInstance;
		DUOResolutionInfo 	duoResolutionInfo;

		char 	duoDeviceName[260];
		char 	duoDeviceSerialNumber[260];
		char 	duoDeviceFirmwareVersion[260];
		char 	duoDeviceFirmwareBuild[260];

		void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData);

		DUO();

};
