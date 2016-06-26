#pragma once

#include "stdafx.h"
#include "NuiGrabberFrame.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

struct Point2d
{
	long  x;
	long  y;
};

class NuiCompressedDepthFrame : public NuiGrabberFrame
{
public:
	NuiCompressedDepthFrame()
		: NuiGrabberFrame()
		, m_nDepthWidth(0)
		, m_nDepthHeight(0)
		, m_pDepthBuffer(nullptr)
		, m_ZoomFactor(1.0f)
	{
		m_ViewOffset.x = 0;
		m_ViewOffset.y = 0;
	}
	virtual ~NuiCompressedDepthFrame();

	virtual void	Clear() override;
	void			DeepCopy (const NuiCompressedDepthFrame& other);
	NuiCompressedDepthFrame (const NuiCompressedDepthFrame& other){ DeepCopy(other); }
	NuiCompressedDepthFrame& operator = (const NuiCompressedDepthFrame& other) {	DeepCopy(other); return *this; }

	UINT16*			AllocateBuffer(UINT width, UINT height);
	UINT16*			GetBuffer() const { return m_pDepthBuffer; }
	UINT			GetWidth() const {	return m_nDepthWidth; }
	UINT			GetHeight() const { return m_nDepthHeight; }
	UINT			GetBufferNum() const { return (m_nDepthWidth * m_nDepthHeight); }
	UINT			GetBufferSize() const { return (sizeof(UINT16) * m_nDepthWidth * m_nDepthHeight); }

	float			GetZoomFactor() const { return(m_ZoomFactor); }
	Point2d*		GetViewOffSet() { return(&m_ViewOffset); }

	void			ReadFrameLock() { m_frameMutex.lock_shared(); }
	void			ReadFrameUnlock() { m_frameMutex.unlock_shared(); }
	void			WriteFrameLock() { m_frameMutex.lock(); }
	void			WriteFrameUnlock() { m_frameMutex.unlock(); }

private:
	UINT16*			m_pDepthBuffer;
	UINT			m_nDepthWidth;
	UINT			m_nDepthHeight;

	float			m_ZoomFactor;   // video frame zoom factor (it is 1.0f if there is no zoom)
	Point2d			m_ViewOffset;   // Offset of the view from the top left corner.

	boost::shared_mutex		m_frameMutex;
};