#pragma once

#include "../../NuiTrackerConfig.h"
#include "../NuiKinfuTracker.h"

#include <Eigen/Core>

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;


class NuiKinfuCPUDepthTracker : public NuiKinfuTracker
{
public:
	NuiKinfuCPUDepthTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuCPUDepthTracker();

	virtual bool	EvaluateFrame(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState) override;
	virtual bool	EstimatePose(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint) override;
	virtual void	FeedbackPose(NuiKinfuCameraState* pCameraState) override;
	virtual void	FeedbackPose(NuiKinfuCameraState* pCameraState, NuiKinfuScene* pScene) override;

	virtual bool	VerticesToMappablePosition(NuiCLMappableData* pMappableData) override;
	virtual bool	BufferToMappableTexture(NuiCLMappableData* pMappableData, TrackerBufferType bufferType) override;

	virtual bool	log(const std::string& fileName) const override;

	virtual float	getError() const override { return m_error; }
	virtual int		getCount() const override { return m_numValidPoints; }

protected:
	void	AcquireBuffers();
	void	ReleaseBuffers();

	void	SmoothDepths(float* floatDepths);
	void	SubSampleDepths();
	void	Depth2vertex(NuiCameraIntrinsics cameraIntrics);
	void	Vertex2Normal();
	bool	IterativeClosestPoint(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint);

protected:
	std::vector<NuiFloatImage*>		m_depthsHierarchy;
	std::vector<NuiFloat3Image*>	m_verticesHierarchy;
	NuiFloat3Image					m_normals;
	NuiFloat3Image					m_verticesPrev;
	NuiFloat3Image					m_normalsPrev;

	NuiTrackerConfig m_configuration;
	UINT m_nWidth, m_nHeight;

	float m_error;
	int m_numValidPoints;
};