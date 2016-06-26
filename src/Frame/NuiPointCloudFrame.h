#pragma once

#include "NuiGrabberFrame.h"
#include "Shape\NuiDevicePointCloud.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

class NuiPointCloudFrame : public NuiGrabberFrame
{
public:
	NuiPointCloudFrame();
	virtual ~NuiPointCloudFrame();

	virtual void		Clear() override;
	NuiDevicePoint*		AllocatePoints(UINT num);
	UINT				GetPointsNum() const { return m_pointCloud.GetPointsNum(); }
	void				SetWidthStep(UINT width) { m_pointCloud.SetWidthStep(width); }
	NuiDevicePoint*		AccessPoint(UINT id) const;
	bool				ReadPointCloud(NuiDevicePointCloud* pPointCloud) const;

	void				SetColorImage(const NuiColorImage& image) { m_pointCloud.SetColorImage(image); }
	const NuiColorImage&	GetColorImage() const { return m_pointCloud.GetColorImage(); }

	void				ReadFrameLock() { m_frameMutex.lock_shared(); }
	void				ReadFrameUnlock() { m_frameMutex.unlock_shared(); }
	void				WriteFrameLock() { m_frameMutex.lock(); }
	void				WriteFrameUnlock() { m_frameMutex.unlock(); }

private:
	NuiDevicePointCloud	m_pointCloud;

	boost::shared_mutex	m_frameMutex;
};