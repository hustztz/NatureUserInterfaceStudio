#pragma once

#include "NuiKinfuVolume.h"
#include "NuiKinfuVolumeConfig.h"
#include "NuiKinfuPointCloud.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"
#include "Kernels/gpu_def.h"

class NuiKinfuTSDFVolume : public NuiKinfuVolume
{
public:
	NuiKinfuTSDFVolume(const NuiKinfuVolumeConfig& config);
	~NuiKinfuTSDFVolume();

	/** \brief Resets tsdf volume data to uninitialized state */
	virtual void	reset() override;
	virtual bool	log(const std::string& fileName) const override;
	virtual bool	hasColorData() const override { return (m_colorVolumeCL ?  true : false); }

	virtual void	integrateVolume(
		cl_mem floatDepthsCL,
		cl_mem normalsCL,
		cl_mem colorsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		UINT nWidth, UINT nHeight
		) override;
	virtual void    raycastRender(
		cl_mem renderVerticesCL,
		cl_mem renderNormalsCL,
		cl_mem renderColorsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		UINT nWidth, UINT nHeight
		) override;

	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) override;

	/** \brief Returns volume size in meters */
	const Vector3f&	getDimensions() const;

	/** \brief Returns volume resolution */
	const Vector3i&	getResolution() const;

	/** \brief Returns volume voxel size in meters */
	const Vector3f	getVoxelSize() const;

	/** \brief Returns tsdf truncation distance in meters */
	float	getTsdfTruncDist () const;

protected:
	void			AcquireBuffer(bool bHas_color_volume);
	void			ReleaseBuffer();

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

private:
	/** \brief tsdf volume data container */
	cl_mem				m_volumeCL;
	/** \brief tsdf volume data container */
	cl_mem				m_colorVolumeCL;

	cl_mem				m_volume_paramsCL;

	cl_mem				m_MB_numVertsTableCL;
	cl_mem				m_MB_triTableCL;

	TsdfParams			m_tsdf_params;

	NuiKinfuVolumeConfig m_config;

	Vector3i			m_voxel_offset;

	NuiKinfuPointCloud	m_cachedPointCloud;
};