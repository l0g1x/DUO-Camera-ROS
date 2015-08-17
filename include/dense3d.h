#ifndef DUOCamera_Dense3D_h
#define DUOCamera_Dense3D_h

#include <Dense3D.h>
#include <ros/ros.h>

namespace duoStereo_driver
{

class Dense3D 
{

public:

	Dense3D(void);
	~Dense3D(void);

	static Dense3D&	GetInstance(void)
	{
		if( pSingleton == 0L )
			pSingleton = new Dense3D();

		return *pSingleton;
	}

	static void	DestroyInstance(void)
	{
		if(pSingleton != 0L)
		{
			pSingleton->shutdownDense3D();

			delete pSingleton;
			pSingleton = NULL;
		}
	}

	void initializeDense3D(int licenseNumber);
	void shutdownDense3D();

protected:
	Dense3DInstance _dense3d;

	static Dense3D* pSingleton;
};
}
#endif