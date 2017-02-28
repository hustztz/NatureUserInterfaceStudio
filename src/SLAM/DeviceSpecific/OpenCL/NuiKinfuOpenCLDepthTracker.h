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

	virtual bool	EvaluateFrame(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState) override;
	virtual bool	EstimatePose(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint) override;
	virtual void	FeedbackPose(NuiKinfuCameraState* pCameraState) override;
	virtual void	FeedbackPose(NuiKinfuCameraState* pCameraState, NuiKinfuScene* pScene) override;

	virtual bool	VerticesToMappablePosition(NuiCLMappableData* pMappableData) override;
	virtual bool	BufferToMappableTexture(NuiCLMappableData* pMappableData, BufferType bufferType) override;

	virtual bool	log(const std::string& fileName) const override;

	virtual float	getError() const override { return m_error; }
	virtual float	getCount() const override { return m_count; }

	cl_mem	getNormalsCL() const { return m_normalsHierarchyCL[0]; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();

	void	GenerateGaussianBuffer();
	void	SmoothDepths(cl_mem floatDepthsCL);
	void	SubSampleDepths();
	void	Depth2vertex(cl_mem cameraParamsCL);
	void	Vertex2Normal();
	bool	IterativeClosestPoint(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint);

protected:
	cl_mem m_gaussianCL;
	typedef std::vector<cl_mem> HierarchyBuffers;
	HierarchyBuffers m_depthsHierarchyCL;
	HierarchyBuffers m_verticesHierarchyCL;
	HierarchyBuffers m_normalsHierarchyCL;
	cl_mem m_verticesPrevCL;
	cl_mem m_normalsPrevCL;
	cl_mem m_corespsBlocksCL;
	cl_mem m_corespsCL;

	NuiTrackerConfig m_configuration;
	std::vector<UINT> m_iterations;
	UINT m_nWidth, m_nHeight;

	float m_error;
	float m_count;
};