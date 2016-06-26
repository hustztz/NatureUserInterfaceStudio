#pragma once

#include <Windows.h>

// Forwards

class NuiKinectV2Grabber
{
public:
	NuiKinectV2Grabber(IKinectSensor* pNuiSensor);
	~NuiKinectV2Grabber();

	bool								initializeDevice();

private:
	NuiKinectV2Grabber (const NuiKinectV2Grabber&); // Disabled copy constructor
	NuiKinectV2Grabber& operator = (const NuiKinectV2Grabber&); // Disabled assignment operator

private:
	//boost::shared_ptr<boost::thread>	m_Thread;
	bool								m_threadOn;
	bool								m_threadPause;

	IKinectSensor*						m_pNuiSensor;
	ICoordinateMapper*					m_pMapper;

	unsigned short						m_minDepth;
	unsigned short						m_maxDepth;
};