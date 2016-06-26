#pragma once

#include <QtGui/QWidget>
#include <QtCore/QTimer.h>

#include <memory>

typedef void (*CacheTimerCallingBack)(void* lpParam);

// Forwards
class NuiFrameCacheImpl;

class NuiMayaCacheTimer : public QWidget
{
	Q_OBJECT
public:
	NuiMayaCacheTimer(CacheTimerCallingBack callBack);
	~NuiMayaCacheTimer();

	void						start(int msec);
	void						stop();

	void						updateCache(NuiFrameCacheImpl* pBuffer) { m_pBuffer = pBuffer; }

public slots:
	void						timer_handler();

private:
	std::shared_ptr<QTimer>		mTimer;

	CacheTimerCallingBack		m_CallBack;
	NuiFrameCacheImpl*			m_pBuffer;

private:
	// privates these
	NuiMayaCacheTimer(const NuiMayaCacheTimer&) {}
	NuiMayaCacheTimer& operator=(const NuiMayaCacheTimer&) { return *this; }
};