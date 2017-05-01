#pragma once

#include "NuiRGBDDeviceCircleBuffer.h"

// Forwards
class NuiRGBDDeviceManagerImpl;

class NuiRGBDDeviceController
{
public:
	enum NuiGrabberDeviceMode
	{
		EDeviceMode_VertexColorCamera		= 0,
		EDeviceMode_Color					= 1,
		EDeviceMode_Vertex					= 2,
		EDeviceMode_Skeleton				= 3,
		EDeviceMode_DepthColor				= 4,
		EDeviceMode_VertexColorPlayer		= 5,
		EDeviceMode_VertexColorSkeleton		= 6,
		EDeviceMode_VertexColorSkeletonFace	= 7,
		EDeviceMode_VertexColorSkeletonGesture	= 8,
		EDeviceMode_Fusion					= 9,

		EDeviceMode_LastFlag				= 10,
	};
public:
	NuiRGBDDeviceController();
	~NuiRGBDDeviceController();

	bool initializeDevice(DWORD deviceMode);
	bool initializeFileLoader(const std::string&	fileName);
	bool startDevice();
	void pauseDevice();
	void stopDevice();

	std::shared_ptr<NuiCompositeFrame>	popFrame();

	bool			UpdateNearMode(bool bNearMode);
	void			UpdateElevationAngle(long tiltAngle);

private:

	NuiRGBDDeviceManagerImpl*		m_pDevice;
	NuiRGBDDeviceCircleBuffer		m_buffer;
	DWORD						m_bufferFlags;
};