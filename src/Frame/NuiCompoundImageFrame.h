#pragma once

#include "NuiGrabberFrame.h"
#include "Shape/NuiCompoundImage.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

class NuiCompoundImageFrame : public NuiGrabberFrame
{
public:
	NuiCompoundImageFrame();
	virtual ~NuiCompoundImageFrame();

	virtual void		Clear() override;
	NuiCompoundPixel*	AllocatePixels(UINT w, UINT h);
	UINT				GetImageWidth() const { return m_compoundImage.GetWidth(); }
	UINT				GetImageHeight() const { return m_compoundImage.GetHeight(); }
	bool				ReadPixel(UINT w, UINT h, NuiCompoundPixel* pPixel) const;

	void				ReadFrameLock() { m_frameMutex.lock_shared(); }
	void				ReadFrameUnlock() { m_frameMutex.unlock_shared(); }
	void				WriteFrameLock() { m_frameMutex.lock(); }
	void				WriteFrameUnlock() { m_frameMutex.unlock(); }

private:
	NuiCompoundImage	m_compoundImage;

	boost::shared_mutex	m_frameMutex;
};