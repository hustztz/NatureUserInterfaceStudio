#pragma once

#include "../NuiKinfuScene.h"
#include "../../NuiKinfuVolumeConfig.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"
#include <Foundation/SgVec3T.h>
#include "Kernels/gpu_def.h"

class NuiKinfuOpenCLScene : public NuiKinfuScene
{
public:
	NuiKinfuOpenCLScene(const NuiKinfuVolumeConfig& config);
	virtual ~NuiKinfuOpenCLScene();

	/** \brief Resets tsdf volume data to uninitialized state */
	virtual void	reset() override;
	virtual bool	log(const std::string& fileName) const override;
	virtual bool	hasColorData() const override { return (m_colorVolumeCL ?  true : false); }
	virtual float	getVoxelLeafSize() const override;

	virtual bool	integrateVolume(
		NuiKinfuFrame*			pFrame,
		NuiKinfuCameraState*	pCameraState
		) override;

	virtual void	raycastRender(
		NuiKinfuFeedbackFrame*	pFeedbackFrame,
		NuiKinfuCameraState*	pCameraState
		) override;

	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) override;

protected:
	virtual Vector3i getVoxelWrap() const {
		return Vector3i(0, 0, 0);
	}

public:
	/** \brief Returns volume size in meters */
	const Vector3f&	getDimensions() const;

	/** \brief Returns volume resolution */
	const Vector3i&	getResolution() const;

	/** \brief Returns volume voxel size in meters */
	const Vector3f	getVoxelSize() const;

	/** \brief Returns tsdf truncation distance in meters */
	float			getTsdfTruncDist () const;

protected:
	void			AcquireBuffer(bool bHas_color_volume);
	void			ReleaseBuffer();

	int				FetchVolume(NuiCLMappableData* pCLData);

	SgVec3f			getNodeCoo(int x, int y, int z);

protected:
	cl_mem				m_vertexSumCL;
	/** \brief tsdf volume data container */
	cl_mem				m_volumeCL;
	/** \brief tsdf volume data container */
	cl_mem				m_colorVolumeCL;

	cl_mem				m_volume_paramsCL;

	cl_mem				m_MB_numVertsTableCL;
	cl_mem				m_MB_triTableCL;

	TsdfParams			m_tsdf_params;

	NuiKinfuVolumeConfig m_config;
};