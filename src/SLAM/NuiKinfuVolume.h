#pragma once

#include "stdafx.h"
#include "NuiKinfuPointCloud.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"
#include "Kernels/gpu_def.h"

#include <atomic>

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector3i Vector3i;

//Forwards
class NuiCLMappableData;
class NuiMeshShape;

class NuiKinfuVolume
{
public:
	NuiKinfuVolume(const Vector3i& resolution, const Vector3f& dimensions, bool bHas_color_volume);
	~NuiKinfuVolume();

    /** \brief Sets Tsdf truncation distance. Must be greater than 2 * volume_voxel_size
    * \param[in] distance TSDF truncation distance 
    */
    void setTsdfTruncDist (float distance);

	void setDynamicOffset (bool bIsDynamic, int voxel_shift);

	void setTranslateBasis(const Vector3f& basis) { m_translateBasis = basis; }

	/** \brief Returns volume size in meters */
	const Vector3f&	getDimensions() const;

	/** \brief Returns volume resolution */
	const Vector3i&	getResolution() const;

	/** \brief Returns volume voxel size in meters */
	const Vector3f	getVoxelSize() const;

	/** \brief Returns tsdf truncation distance in meters */
	float	getTsdfTruncDist () const;

	/** \brief Returns running average length */
	unsigned char	getMaxColorWeight() const { return m_max_color_weight; }

	Vector3i getVoxelWrap() const;
	Vector3f getVoxelOffsetSize() const;

	/** \brief Returns tsdf volume container that point to data in GPU memroy */
	cl_mem data() const;

	/** \brief Returns tsdf volume container that point to data in GPU memroy */
	cl_mem colorData() const;

	cl_mem paramData() const { return m_volume_paramsCL; }

	bool hasColorData() const { return (NULL != m_colorVolumeCL); }

	/** \brief Resets tsdf volume data to uninitialized state */
	void reset();

	void setDirty() { m_dirty = true; }

	Vector3f shiftVolume(const Vector3f& translation);

	bool	Volume2CLVertices(NuiCLMappableData* pCLData);
	bool	Volume2CLMesh(NuiCLMappableData* pCLData);
	bool	Volume2Mesh(NuiMeshShape* pMesh);

protected:
	void	AcquireBuffer(bool bHas_color_volume);
	void	ReleaseBuffer();

	SgVec3f	getNodeCoo(int x, int y, int z);
	Vector3i vWrap();
	void clearSlice(const Vector3i&	voxelWrap, const Vector3i&	voxelRange);
	void fetchSlice(const Vector3i&	voxelWrap, const Vector3i&	voxelRange);

	void fetchAndClearX(int xVoxelTrans);
	void fetchAndClearY(int yVoxelTrans);
	void fetchAndClearZ(int zVoxelTrans);

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

	/** \brief tsdf volume size in meters */
	Vector3f	m_dimensions;

	/** \brief tsdf volume resolution */
	Vector3i	m_resolution;

	Vector3i	m_voxel_offset;
	Vector3f	m_translateBasis;

	unsigned char m_max_color_weight;
	int m_max_output_vertex_size;
	bool m_bIsDynamic;
	int	 m_voxel_shift;

	NuiKinfuPointCloud m_cachedPointCloud;

	std::atomic<bool> m_dirty;
};