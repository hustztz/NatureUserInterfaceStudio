#pragma once

#include "NuiTrackerConfig.h"
#include "NuiKinfuTracker.h"

#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;


class NuiKinfuOpenCLDepthTracker : public NuiKinfuTracker
{
public:
	NuiKinfuOpenCLDepthTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuOpenCLDepthTracker();

	virtual bool	trackFrame(NuiKinfuFrameImpl* pFrame, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint) override;
	virtual bool	readFrame(NuiKinfuFrameImpl* pFrame) override;
	virtual void	transformPrevsFrame(NuiKinfuTransform* pTransform) override;
	virtual void	resizePrevsFrame() override;
	virtual void	copyPrevsFrame() override;

	virtual bool	log(const std::string& fileName) const override;

	virtual float	getError() const override { return m_error; }
	virtual float	getCount() const override { return m_count; }

	cl_mem	getNormalsCL() const { return m_normalsArrCL[0]; }
	cl_mem	getPrevVerticesCL() const { return m_verticesPrevArrCL[0]; }
	cl_mem	getPrevNormalsCL() const { return m_normalsPrevArrCL[0]; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();

	void	GenerateGaussianBuffer();
	void	SmoothDepths(cl_mem floatDepthsCL);
	void	ColorsToIntensity(cl_mem colorsCL);
	void	NormalEst(cl_mem cameraParamsCL);
	bool	IterativeClosestPoint(cl_mem cameraParamsCL, NuiKinfuTransform* pTransform, Eigen::Affine3f *hint);

protected:
	cl_mem m_gaussianCL;
	typedef std::vector<cl_mem> GPUBuffers;
	GPUBuffers m_depthsArrCL;
	GPUBuffers m_verticesArrCL;
	GPUBuffers m_normalsArrCL;
	GPUBuffers m_verticesPrevArrCL;
	GPUBuffers m_normalsPrevArrCL;
	cl_mem m_corespsBlocksCL;
	cl_mem m_corespsCL;

	NuiTrackerConfig m_configuration;
	std::vector<UINT> m_iterations;
	UINT m_nWidth, m_nHeight;

	float m_error;
	float m_count;
};