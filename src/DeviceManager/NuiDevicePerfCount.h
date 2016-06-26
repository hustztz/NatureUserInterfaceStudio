#pragma once

#include <Windows.h>
#include <iostream>

class NuiDevicePerfCount
{
public:
	NuiDevicePerfCount()
	{
		LARGE_INTEGER qpf = {0};
		if (QueryPerformanceFrequency(&qpf))
		{
			m_frequency = double(qpf.QuadPart);
		}
		else
		{
			m_frequency = 0.0;
		}
		NuiDevicePerfCount(m_frequency);
	}
	NuiDevicePerfCount(double freq)
		: m_frequency(freq)
	{
		QueryPerformanceCounter(&m_tStartTimeStamp);
	}
	~NuiDevicePerfCount()
	{
		if(m_frequency > 0.0)
		{
			LARGE_INTEGER endTimeStamp = {0};
			QueryPerformanceCounter(&endTimeStamp);

			double processTime = double(endTimeStamp.QuadPart - m_tStartTimeStamp.QuadPart) / m_frequency;
			std::cout << "Time : " << processTime << std::endl;
		}
	}
private:
	LARGE_INTEGER	m_tStartTimeStamp;
	double			m_frequency;
};
