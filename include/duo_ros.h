#include <DUOLib.h>
#include <stdio.h>
#include <ros/ros.h>

bool Initialize();
		
DUOInstance 		duoInstance;
DUOResolutionInfo 	duoResolutionInfo;

char 	duoDeviceName[260];
char 	duoDeviceSerialNumber[260];
char 	duoDeviceFirmwareVersion[260];
char 	duoDeviceFirmwareBuild[260];

bool 	useDUO_Imu;
bool	useDUO_LEDs;

void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData);

