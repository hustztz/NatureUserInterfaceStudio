#pragma once

#include "NuiKinfuOpenCLScene.h"

class NuiKinfuOpenCLShiftScene : public NuiKinfuOpenCLScene
{
public:
	NuiKinfuOpenCLShiftScene(const NuiKinfuVolumeConfig& config);
	virtual ~NuiKinfuOpenCLShiftScene();

	/** \brief Resets tsdf volume data to uninitialized state */
	virtual void	reset() override;
	virtual bool	log(const std::string& fileName) const override;
	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) override;

protected:
	virtual Vector3i getVoxelWrap() const override;
protected:
	void			AcquireBuffer(bool bHas_color_volume);
	void			ReleaseBuffer();

	
	Vector3i vWrap();
	Vector3f getVoxelOffsetSize() const;

	void clearSlice(const Vector3i&	voxelWrap, const Vector3i&	voxelRange);
	void fetchSlice(const Vector3i&	voxelWrap, const Vector3i&	voxelRange);

	void fetchAndClearX(int xVoxelTrans);
	void fetchAndClearY(int yVoxelTrans);
	void fetchAndClearZ(int zVoxelTrans);

	Vector3f shiftVolume(const Vector3f& translation);

private:
	Vector3i			m_voxel_offset;

};