#pragma once

#include "NuiGrabberFrame.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#ifndef _NUI_DEPTH_IMAGE_PIXEL_
#define _NUI_DEPTH_IMAGE_PIXEL_
typedef struct _NUI_DEPTH_IMAGE_PIXEL
{
	unsigned short playerIndex;
	unsigned short depth;
} 	NUI_DEPTH_IMAGE_PIXEL;
#endif _NUI_DEPTH_IMAGE_PIXEL_

class NuiExtendedDepthFrame : public NuiGrabberFrame
{
public:
	NuiExtendedDepthFrame()
		: NuiGrabberFrame()
		, m_nDepthWidth(0)
		, m_nDepthHeight(0)
		, m_pDepthBuffer(nullptr)
	{}
	virtual ~NuiExtendedDepthFrame();

	virtual void	Clear() override;
	void			DeepCopy (const NuiExtendedDepthFrame& other);
	NuiExtendedDepthFrame (const NuiExtendedDepthFrame& other){ DeepCopy(other); }
	NuiExtendedDepthFrame& operator = (const NuiExtendedDepthFrame& other) {	DeepCopy(other); return *this; }

	NUI_DEPTH_IMAGE_PIXEL*			AllocateBuffer(UINT width, UINT height);
	NUI_DEPTH_IMAGE_PIXEL*			GetBuffer() const { return m_pDepthBuffer; }
	UINT			GetWidth() const {	return m_nDepthWidth; }
	UINT			GetHeight() const { return m_nDepthHeight; }
	UINT			GetBufferNum() const { return (m_nDepthWidth * m_nDepthHeight); }
	UINT			GetBufferSize() const { return (sizeof(NUI_DEPTH_IMAGE_PIXEL) * m_nDepthWidth * m_nDepthHeight); }

	void			ReadFrameLock() { m_frameMutex.lock_shared(); }
	void			ReadFrameUnlock() { m_frameMutex.unlock_shared(); }
	void			WriteFrameLock() { m_frameMutex.lock(); }
	void			WriteFrameUnlock() { m_frameMutex.unlock(); }

private:
	NUI_DEPTH_IMAGE_PIXEL*	m_pDepthBuffer;
	UINT			m_nDepthWidth;
	UINT			m_nDepthHeight;

	boost::shared_mutex		m_frameMutex;
};