#pragma once

#include <thread>   // std::thread

typedef void (*ThreadCallingBack)(void* lpParam);

class NuiThreadManager
{
public:
	NuiThreadManager()
		: m_Thread(m_CallBack)
	{}
	~NuiThreadManager()
	{
		m_Thread.join();
	}

protected:
	void threadFunc()
	{
		if (m_CallBack)
		{
			(*m_CallBack)(m_pCache);
		}
	}

private:
	NuiThreadManager (const NuiThreadManager&); // Disabled copy constructor
	NuiThreadManager& operator = (const NuiThreadManager&); // Disabled assignment operator

private:
	std::thread			m_Thread;

	ThreadCallingBack	m_CallBack;
};