#include "NuiThreadObject.h"

NuiThreadObject::NuiThreadObject()
	: m_threadOn(false)
	, m_threadPause(false)
{

}

NuiThreadObject::~NuiThreadObject()
{
	stopThread ();
}

void	NuiThreadObject::startThread ()
{
	if(!m_threadOn)
	{
		reset();
		m_Thread.reset (new boost::thread (boost::bind (&NuiThreadObject::runThread, this)));
		m_threadOn = true;

	}
	m_threadPause = false;
}

void	NuiThreadObject::stopThread()
{
	m_threadOn = false;
	m_threadPause = false;
	if(m_Thread)
	{
		//m_Thread->interrupt();
		m_Thread->join();
	}
}

void	NuiThreadObject::waitThread()
{
	if(m_Thread)
	{
		//m_Thread->interrupt();
		m_Thread->join();
	}
}

void	NuiThreadObject::pauseThread()
{
	m_threadPause = true;
}

void	NuiThreadObject::runThread ()
{
	while (true)
	{
		if (!m_threadOn)
			break;
		if (m_threadPause)
			continue;

		if ( !process() )
		{
			break;
		}
	}
	m_threadOn = false;
}
