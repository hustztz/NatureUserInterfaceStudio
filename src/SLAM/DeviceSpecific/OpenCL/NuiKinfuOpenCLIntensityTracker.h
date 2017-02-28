#pragma once

#include "NuiKinfuOpenCLDepthTracker.h"

class NuiKinfuOpenCLIntensityTracker : public NuiKinfuOpenCLDepthTracker
{
public:
	NuiKinfuOpenCLIntensityTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuOpenCLIntensityTracker();

	virtual bool	EvaluateFrame(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState) override;
	virtual bool	EstimatePose(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint) override;
	virtual void	FeedbackPose(NuiKinfuCameraState* pCameraState) override;

	virtual bool	previousBufferToData(NuiCLMappableData* pMappableData) override;

	virtual bool	hasColorData() const override { return true; }

	cl_mem	getIntensitiesCL() const { return m_intensitiesArrCL[0]; }
	cl_mem	getPrevIntensitiesCL() const { return m_intensitiesPrevArrCL[0]; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();

	void	resizePrevsMaps();
	void	ColorsToIntensity(cl_mem colorsCL);
	bool	IntensityIterativeClosestPoint(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint);
	void	CopyPrevIntensityMaps();

protected:
	HierarchyBuffers m_intensitiesArrCL;
	HierarchyBuffers m_intensitiesPrevArrCL;
	HierarchyBuffers m_intensityDerivsPrevArrCL;
};