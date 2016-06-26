#pragma once

#include "NuiGrabberFrame.h"
#include "Shape\NuiImageBuffer.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

class NuiDepthFrame : public NuiGrabberFrame
{
public:
	NuiDepthFrame()
		: NuiGrabberFrame()
		, m_nDepthFPS(0)
		, m_minDepth(500)
		, m_maxDepth(4500)
	{}
	virtual ~NuiDepthFrame();

	virtual void	Clear() override;
	void			DeepCopy (const NuiDepthFrame& other);
	NuiDepthFrame (const NuiDepthFrame& other){ DeepCopy(other); }
	NuiDepthFrame& operator = (const NuiDepthFrame& other) {	DeepCopy(other); return *this; }

	UINT16*			AllocateBuffer(UINT width, UINT height) { return m_depthImage.AllocateBuffer(width, height); }
	UINT16*			GetBuffer() const { return m_depthImage.GetBuffer(); }
	UINT			GetWidth() const {	return m_depthImage.GetWidth(); }
	UINT			GetHeight() const { return m_depthImage.GetHeight(); }
	UINT			GetBufferNum() const { return m_depthImage.GetBufferNum(); }
	UINT			GetBufferSize() const { return m_depthImage.GetBufferSize(); }

	const NuiDepthImage&	GetImage() const { return m_depthImage; }

	void			SetFPS(double fps) { m_nDepthFPS = fps; }
	double			GetFPS() const { return m_nDepthFPS; }

	void		SetMinDepth(UINT16 depth) { m_minDepth = depth; }
	UINT16		GetMinDepth() const { return m_minDepth; }
	void		SetMaxDepth(UINT16 depth) { m_maxDepth = depth; }
	UINT16		GetMaxDepth() const { return m_maxDepth; }

	void			ReadFrameLock() { m_frameMutex.lock_shared(); }
	void			ReadFrameUnlock() { m_frameMutex.unlock_shared(); }
	void			WriteFrameLock() { m_frameMutex.lock(); }
	void			WriteFrameUnlock() { m_frameMutex.unlock(); }

private:
	NuiDepthImage	m_depthImage;
	double			m_nDepthFPS;
	UINT16			m_minDepth;
	UINT16			m_maxDepth;

	boost::shared_mutex		m_frameMutex;
};