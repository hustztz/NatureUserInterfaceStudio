#pragma once

#include "NuiTrackerConfig.h"
#include "NuiKinfuOpenCLDepthTracker.h"

class NuiKinfuOpenCLIntensityTracker : public NuiKinfuOpenCLDepthTracker
{
public:
	NuiKinfuOpenCLIntensityTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuOpenCLIntensityTracker();

	virtual bool	trackFrame(NuiKinfuFrameImpl* pFrame, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint) override;
	virtual bool	readFrame(NuiKinfuFrameImpl* pFrame) override;
	virtual void	transformPrevsFrame(NuiKinfuTransform* pTransform) override;
	virtual void	resizePrevsFrame() override;
	virtual void	copyPrevsFrame() override;

	virtual bool	hasColorData() const override { return true; }

	cl_mem	getIntensitiesCL() const { return m_intensitiesArrCL[0]; }
	cl_mem	getPrevIntensitiesCL() const { return m_intensitiesPrevArrCL[0]; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();

	void	ColorsToIntensity(cl_mem colorsCL);
	bool	IntensityIterativeClosestPoint(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint);
	void	CopyPrevIntensityMaps();

protected:
	GPUBuffers m_intensitiesArrCL;
	GPUBuffers m_intensitiesPrevArrCL;
	GPUBuffers m_intensityDerivsPrevArrCL;
};