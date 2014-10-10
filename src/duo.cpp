#include "duo_ros.h"


DUO::DUO(void)
{

}

void DUO::Initialize()
{
	if(EnumerateResolutions(&duoResolutionInfo, 1, 320, 240, DUO_BIN_HORIZONTAL2+DUO_BIN_VERTICAL2, 30))
	{
		if(OpenDUO(&duoInstance))
		{
			SetDUOResolutionInfo( duoInstance, duoResolutionInfo);
		}
		else
		{
			ROS_ERROR("Cannot Open DUO. Please check connection!");
		}
	}
}

void DUO::DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{

}

void CALLBACK DUOCallback(const PDUOFrame pFrameData, void *pUserData)
{
    printf("  Timestamp:          %10.1f ms\n", pFrameData->timeStamp/10.0f);
    printf("  Frame Size:         %dx%d\n", pFrameData->width, pFrameData->height);
    printf("  Left Frame Buffer:  %p\n", pFrameData->leftData);
    printf("  Right Frame Buffer: %p\n", pFrameData->rightData);
    printf("------------------------------------------------------\n");
}

int main(int argc, char** argv)
{
    	ros::init(argc, argv, "duo3d_camera");
	ros::NodeHandle n;


	printf("DUOLib Version:       v%s\n", GetLibVersion());

    DUOResolutionInfo ri;
    // Select 320x240 resolution with 2x2 binning capturing at 10FPS
    if(EnumerateResolutions(&ri, 1, 320, 240, DUO_BIN_HORIZONTAL2+DUO_BIN_VERTICAL2, 30))
    {
        DUOInstance duo;
        // Open DUO
        if(OpenDUO(&duo))
        {
            char tmp[260];
            // Get some DUO parameter values
            GetDUODeviceName(duo, tmp);
            printf("DUO Device Name:      '%s'\n", tmp);
            GetDUODeviceName(duo, tmp);
            printf("DUO Serial Number:    %s\n", tmp);
            GetDUOFirmwareVersion(duo, tmp);
            printf("DUO Firmware Version: v%s\n", tmp);
            GetDUOFirmwareBuild(duo, tmp);
            printf("DUO Firmware Build:   %s\n", tmp);

            printf("\nHit any key to start capturing");

            // Set selected resolution
            SetDUOResolutionInfo(duo, ri);
            // Start capture and pass DUOCallback function that will be called on every frame captured
            if(StartDUO(duo, DUOCallback, NULL))
            {
                // Wait for any key
                // Stop capture
                StopDUO(duo);
                // Close DUO
                CloseDUO(duo);
            }
        }
	else
	{
		ROS_ERROR("Cannot Open DUO!");
	}
    }




	//DUO::Initialize(void);

	return 0;
}
