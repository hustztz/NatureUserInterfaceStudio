#include "NuiMayaCacheTimer.h"

#include "Frame/Buffer/NuiFrameCacheImpl.h"


NuiMayaCacheTimer::NuiMayaCacheTimer(CacheTimerCallingBack callBack)
	: m_CallBack(callBack)
	, m_pBuffer(NULL)
{
	mTimer = std::shared_ptr<QTimer>(new QTimer(this));
	connect(mTimer.get(), SIGNAL(timeout()), this, SLOT(timer_handler()));
}

NuiMayaCacheTimer::~NuiMayaCacheTimer()
{
	mTimer->stop();
}

void NuiMayaCacheTimer::timer_handler()
{
	if (!m_pBuffer)
		return;

	//Refresh Preview
	if (m_CallBack)
	{
		(*m_CallBack)(m_pBuffer);
	}
}

void NuiMayaCacheTimer::start(int msec)
{
	if (mTimer->isActive()) {
		printf_s("Neuron timer interval changed to %d.\n", msec);
		mTimer->setInterval(msec);
	}
	else {
		printf_s("Neuron timer started with interval = %d.\n", msec);
		mTimer->start(msec);
	}
}

void NuiMayaCacheTimer::stop()
{
	mTimer->stop();
}