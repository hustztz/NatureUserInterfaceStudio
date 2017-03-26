#pragma once

#include <boost/thread/thread.hpp>

class NuiThreadObject
{
public:
	NuiThreadObject();
	virtual ~NuiThreadObject();

	virtual void						reset() {}
	void								startThread ();
	void								stopThread();
	void								pauseThread();
	void								stepIn();
	void								waitThread();
	bool								isThreadOn() const { return m_threadOn; }

protected:
	void								runThread ();
	virtual bool						process ()
	{
		assert(false);
		return false;
	}

protected:
	bool								m_threadPause;

private:
	boost::shared_ptr<boost::thread>	m_Thread;
	bool								m_threadOn;
};