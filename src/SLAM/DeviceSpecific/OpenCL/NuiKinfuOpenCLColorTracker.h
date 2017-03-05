#pragma once

#include "../../NuiTrackerConfig.h"
#include "../NuiKinfuTracker.h"

#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <Eigen/Core>

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;

class NuiKinfuOpenCLColorTracker : public NuiKinfuTracker
{
public:
	NuiKinfuOpenCLColorTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuOpenCLColorTracker();

	virtual bool	EstimatePose(
		NuiKinfuFrame* pFrame,
		NuiKinfuFeedbackFrame* pFeedbackFrame,
		NuiKinfuCameraState* pCameraState,
		Eigen::Affine3f *hint
		) override;

	virtual bool	hasColorData() const override { return true; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();

	void	SubSampleColors(cl_mem colorsCL);
	void	GradientBuffers();
	float	GetColorsDifference(
		int level_index,
		cl_mem vertices,
		cl_mem colors,
		cl_mem level_colors,
		cl_mem cameraParamsCL,
		const Matrix3frm& rot,
		const Vector3f& trans);
	bool	ColorIterativeClosestPoint(
		cl_mem verticesPrevCL,
		cl_mem colorsPrevCL,
		NuiKinfuCameraState* pCameraState,
		Eigen::Affine3f *hint);

protected:
	typedef std::vector<cl_mem> HierarchyBuffers;
	HierarchyBuffers m_colorsArrCL;
	HierarchyBuffers m_gradientXArrCL;
	HierarchyBuffers m_gradientYArrCL;

	cl_mem m_corespsBlocksCL;
	cl_mem m_corespsCL;

	cl_mem m_colorDiffCL;
	cl_mem m_numValidCL;

	NuiTrackerConfig m_configuration;
	UINT m_nWidth, m_nHeight;

	float m_count;
};