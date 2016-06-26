#pragma once

#include "NuiGrabberFrame.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

class NuiFusionImageFrame : public NuiGrabberFrame
{
public:
	NuiFusionImageFrame()
		: NuiGrabberFrame()
		, m_nWidth(0)
		, m_nHeight(0)
		, m_nFPS(0)
		, m_pBuffer(nullptr)
	{}
	virtual ~NuiFusionImageFrame();

	virtual void	Clear() override;
	void			DeepCopy (const NuiFusionImageFrame& other);
	NuiFusionImageFrame (const NuiFusionImageFrame& other){ DeepCopy(other); }
	NuiFusionImageFrame& operator = (const NuiFusionImageFrame& other) {	DeepCopy(other); return *this; }

	BGRQUAD*		AllocateBuffer(UINT width, UINT height);
	BGRQUAD*		GetBuffer() const { return m_pBuffer; }
	UINT			GetWidth() const {	return m_nWidth; }
	UINT			GetHeight() const { return m_nHeight;	}
	UINT			GetBytesPerPixel() const { return sizeof(BGRQUAD); }
	UINT			GetBufferSize() const { return (sizeof(BGRQUAD) * m_nWidth * m_nHeight); }
	void			SetFPS(double fps) { m_nFPS = fps; }
	double			GetFPS() const { return m_nFPS; }

	void			ReadFrameLock() { m_frameMutex.lock_shared(); }
	void			ReadFrameUnlock() { m_frameMutex.unlock_shared(); }
	void			WriteFrameLock() { m_frameMutex.lock(); }
	void			WriteFrameUnlock() { m_frameMutex.unlock(); }

private:
	BGRQUAD*		m_pBuffer;
	UINT			m_nWidth;
	UINT			m_nHeight;
	double			m_nFPS;

	boost::shared_mutex	m_frameMutex;
};