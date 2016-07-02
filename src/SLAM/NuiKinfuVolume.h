#pragma once

#include "stdafx.h"
#include "NuiKinfuVolumeConfig.h"
#include "NuiKinfuPointCloud.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"
#include "Kernels/gpu_def.h"

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
	NuiKinfuVolume(const NuiKinfuVolumeConfig& config);
	~NuiKinfuVolume();

	/** \brief Returns volume size in meters */
	const Vector3f&	getDimensions() const;

	/** \brief Returns volume resolution */
	const Vector3i&	getResolution() const;

	/** \brief Returns volume voxel size in meters */
	const Vector3f	getVoxelSize() const;

	/** \brief Returns tsdf truncation distance in meters */
	float	getTsdfTruncDist () const;

	void setIntegrationMetricThreshold(float threshold) { m_integration_metric_threshold = threshold; }

	bool hasColorData() const { return (NULL != m_colorVolumeCL); }

	/** \brief Resets tsdf volume data to uninitialized state */
	void reset();

	void setDirty() { m_dirty = true; }

	void	incrementVolume(cl_mem floatDepthsCL, cl_mem colorsCL, cl_mem normalsCL, cl_mem cameraParamsCL, const NuiKinfuTransform& currPos, UINT nWidth, UINT nHeight);
	bool	evaluateVolume(cl_mem floatDepthsCL, cl_mem colorsCL, cl_mem normalsCL, cl_mem renderVertices, cl_mem renderNormals, cl_mem cameraParamsCL, const NuiKinfuTransform& currPos, UINT nWidth, UINT nHeight);

	bool	Volume2CLVertices(NuiCLMappableData* pCLData);
	bool	Volume2CLMesh(NuiCLMappableData* pCLData);
	bool	Volume2Mesh(NuiMeshShape* pMesh);

protected:
	void	AcquireBuffer(bool bHas_color_volume);
	void	ReleaseBuffer();

	SgVec3f	getNodeCoo(int x, int y, int z);
	Vector3i vWrap();

	Vector3i getVoxelWrap() const;
	Vector3f getVoxelOffsetSize() const;

	void clearSlice(const Vector3i&	voxelWrap, const Vector3i&	voxelRange);
	void fetchSlice(const Vector3i&	voxelWrap, const Vector3i&	voxelRange);

	void fetchAndClearX(int xVoxelTrans);
	void fetchAndClearY(int yVoxelTrans);
	void fetchAndClearZ(int zVoxelTrans);

	Vector3f shiftVolume(const Vector3f& translation);
	void    Integrate(cl_mem floatDepthsCL, cl_mem colorsCL, cl_mem normalsCL, cl_mem cameraParamsCL, const NuiKinfuTransform& currPos, UINT nWidth, UINT nHeight);
	void    RayCast(cl_mem renderVertices, cl_mem renderNormals, cl_mem cameraParamsCL, const NuiKinfuTransform& currPos, UINT nWidth, UINT nHeight);

private:
	/** \brief tsdf volume data container */
	cl_mem	m_volumeCL;
	/** \brief tsdf volume data container */
	cl_mem	m_colorVolumeCL;

	cl_mem m_volume_paramsCL;
	// Buffers for output
	cl_mem m_volumeOutputVerticesCL;
	cl_mem m_volumeOutputColorsCL;
	cl_mem m_MB_numVertsTableCL;
	cl_mem m_MB_triTableCL;
	cl_mem m_mutexCL;
	cl_mem m_vertexSumCL;

	TsdfParams	m_tsdf_params;

	NuiKinfuVolumeConfig m_config;

	Matrix3frm			m_lastIntegrationRotation;
	Vector3f			m_lastIntegrationTranslation;
	Vector3i			m_voxel_offset;
	int					m_max_output_vertex_size;
	float				m_integration_metric_threshold;

	NuiKinfuPointCloud	m_cachedPointCloud;

	std::atomic<bool>	m_dirty;
};