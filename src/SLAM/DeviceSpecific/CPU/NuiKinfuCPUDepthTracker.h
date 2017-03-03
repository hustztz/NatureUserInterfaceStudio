#pragma once

#include "../../NuiTrackerConfig.h"
#include "../NuiKinfuTracker.h"
#include "Shape\NuiImageBuffer.h"

#include <Eigen/Core>

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector2f Vector2f;
struct NuiCameraIntrinsics;

class NuiKinfuCPUDepthTracker : public NuiKinfuTracker
{
public:
	NuiKinfuCPUDepthTracker(const NuiTrackerConfig& config, UINT nWidth, UINT nHeight);
	virtual ~NuiKinfuCPUDepthTracker();

	virtual bool	EstimatePose(
		NuiKinfuFrame* pFrame,
		NuiKinfuFeedbackFrame* pFeedbackFrame,
		NuiKinfuCameraState* pCameraState,
		Eigen::Affine3f *hint
		) override;

	virtual bool	log(const std::string& fileName) const override;

	virtual float	getError() const override { return m_error; }
	virtual int		getCount() const override { return m_numValidPoints; }

protected:
	void	AcquireBuffers( UINT nWidth, UINT nHeight);
	void	ReleaseBuffers();

	void	SubSampleDepths(float* filteredDepths);
	void	HierarchyDepth2vertex(NuiCameraIntrinsics cameraIntrics);
	Vector3f InterpolateBilinear_withHoles(const Vector3f* source, Vector2f position, UINT nWidth);
	bool	IterativeClosestPoint(
		Vector3f* verticesBuffer,
		Vector3f* verticesPrevBuffer,
		Vector3f* normalsPrevBuffer,
		UINT	nWidth,
		UINT	nHeight,
		NuiKinfuCameraState* pCameraState,
		Eigen::Affine3f *hint);

protected:
	std::vector<NuiFloatImage*>		m_depthsHierarchy;
	std::vector<NuiFloat3Image*>	m_verticesHierarchy;

	NuiTrackerConfig m_configuration;

	float m_error;
	int m_numValidPoints;
};