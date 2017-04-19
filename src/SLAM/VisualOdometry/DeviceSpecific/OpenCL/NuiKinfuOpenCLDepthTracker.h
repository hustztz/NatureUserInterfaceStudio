#pragma once

#include "../../NuiTrackerConfig.h"
#include "../NuiKinfuTracker.h"

#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <Eigen/Core>

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;


class NuiKinfuOpenCLDepthTracker : public NuiKinfuTracker
{
public:
	NuiKinfuOpenCLDepthTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuOpenCLDepthTracker();

	virtual bool	EstimatePose(
		NuiKinfuFrame* pFrame,
		NuiKinfuFeedbackFrame* pFeedbackFrame,
		NuiKinfuCameraState* pCameraState,
		Eigen::Affine3f *hint
		) override;

	virtual bool	log(const std::string& fileName) const override;

	virtual float	getError() const override { return m_error; }
	virtual int		getCount() const override { return (int)m_count; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();

	void	SubSampleDepths(cl_mem filteredDepths);
	void	HierarchyDepth2vertex(cl_mem cameraParamsCL);
	void	TransformBuffers(cl_mem transformCL);
	bool	IterativeClosestPoint(
		cl_mem verticesCL,
		cl_mem verticesPrevCL,
		cl_mem normalsPrevCL,
		NuiKinfuCameraState* pCameraState,
		Eigen::Affine3f *hint);

protected:
	typedef std::vector<cl_mem> HierarchyBuffers;
	HierarchyBuffers m_depthsHierarchyCL;
	HierarchyBuffers m_verticesHierarchyCL;
	cl_mem m_corespsBlocksCL;
	cl_mem m_corespsCL;

	NuiTrackerConfig m_configuration;
	UINT m_nWidth, m_nHeight;

	float m_error;
	float m_count;
};