#pragma once

#include "stdafx.h"

#include "NuiHashingSDF.h"

#include "Foundation/SgVec3T.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

// Forwards
class NuiKinfuTransform;
class NuiHashingChunkGrid;

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;

class NuiHashingVolume
{
public:
	NuiHashingVolume();
	~NuiHashingVolume();

	void	reset();

	void	incrementVolume(
		cl_mem floatDepthsCL,
		cl_mem colorsCL,
		cl_mem normalsCL,
		cl_mem cameraParamsCL,
		const NuiKinfuTransform& currPos,
		UINT nWidth, UINT nHeight);
	bool	evaluateVolume(
		cl_mem floatDepthsCL,
		cl_mem colorsCL,
		cl_mem normalsCL,
		cl_mem renderVertices,
		cl_mem renderNormals,
		cl_mem cameraParamsCL,
		const NuiKinfuTransform& currPos,
		UINT nWidth, UINT nHeight);

protected:
	void raycastRender(
		NuiHashingSDF* pSDF,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		cl_mem verticesCL,
		cl_mem normalsCL,
		float rayIncrement,
		float thresSampleDist,
		float thresDist,
		float minDepth,
		float maxDepth,
		UINT nWidth, UINT nHeight);
private:
	NuiHashingSDF			m_sdfData;
	NuiHashingChunkGrid*	m_pChunkGrid;

	Matrix3frm				m_lastIntegrationRotation;
	Vector3f				m_lastIntegrationTranslation;

	float					m_integration_metric_threshold;

	// Raycast Params
	float					m_rayIncrement;
	float					m_thresSampleDist;
	float					m_thresDist;
	float					m_minDepth;
	float					m_maxDepth;
};