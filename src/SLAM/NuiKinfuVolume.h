#pragma once

#include "stdafx.h"
#include "NuiKinfuPointCloud.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <Eigen/Geometry>
#include <atomic>

//Forwards
class NuiKinfuTransform;
class NuiCLMappableData;
class NuiMeshShape;

typedef Eigen::Matrix<float, 3, 3, Eigen::RowMajor> Matrix3frm;
typedef Eigen::Vector3f Vector3f;

class NuiKinfuVolume
{
public:
	NuiKinfuVolume();
	~NuiKinfuVolume();

	void setIntegrationMetricThreshold(float threshold) { m_integration_metric_threshold = threshold; }
	void setDirty() { m_dirty = true; }

	virtual bool	log(const std::string& fileName) const = 0;
	virtual bool	hasColorData() const { return true; }
	/** \brief Resets tsdf volume data to uninitialized state */
	virtual void	reset();
	virtual void	incrementVolume(
		cl_mem floatDepthsCL,
		cl_mem colorsCL,
		cl_mem normalsCL,
		cl_mem cameraParamsCL,
		const NuiKinfuTransform& currPos,
		UINT nWidth, UINT nHeight
		) = 0;
	virtual bool	evaluateVolume(
		cl_mem floatDepthsCL,
		cl_mem colorsCL,
		cl_mem normalsCL,
		cl_mem renderVertices,
		cl_mem renderNormals,
		cl_mem cameraParamsCL,
		const NuiKinfuTransform& currPos,
		UINT nWidth, UINT nHeight
		) = 0;

	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) = 0;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) = 0;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) = 0;

protected:
	void			AcquireBuffer(bool bHas_color_volume);
	void			ReleaseBuffer();

protected:
	// Buffers for output
	cl_mem				m_volumeOutputVerticesCL;
	cl_mem				m_volumeOutputColorsCL;
	cl_mem				m_mutexCL;
	cl_mem				m_vertexSumCL;

	Matrix3frm			m_lastIntegrationRotation;
	Vector3f			m_lastIntegrationTranslation;

	int					m_max_output_vertex_size;
	float				m_integration_metric_threshold;
	NuiKinfuPointCloud	m_cachedPointCloud;
	std::atomic<bool>	m_dirty;
};