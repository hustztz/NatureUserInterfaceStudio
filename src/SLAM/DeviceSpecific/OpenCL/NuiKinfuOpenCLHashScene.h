#pragma once

#include "../NuiKinfuScene.h"
#include "NuiKinfuVoxelBlockHash.h"
#include "../../NuiHashingSDFConfig.h"
#include "../../NuiHashingRaycastConfig.h"
#include "../../NuiHashingChunkGridConfig.h"
#include "Foundation/SgVec3T.h"

#include "OpenCLUtilities/NuiMappable.h"
#include "OpenCLUtilities/NuiTextureMappable.h"

// Forwards

class NuiKinfuOpenCLHashScene : public NuiKinfuScene
{
public:
	NuiKinfuOpenCLHashScene(const NuiHashingSDFConfig& sdfConfig, const NuiHashingRaycastConfig& raycastConfig);
	~NuiKinfuOpenCLHashScene();

	virtual void	reset() override;
	virtual bool	log(const std::string& fileName) const override;

	virtual bool	integrateVolume(
		NuiKinfuFrame*			pFrame,
		NuiKinfuFeedbackFrame*	pFeedbackFrame,
		NuiKinfuCameraState*	pCameraState
		) override;

	virtual void	raycastRender(
		NuiKinfuFeedbackFrame*	pFeedbackFrame,
		NuiKinfuCameraState*	pCameraState
		) override;
	
	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) override;
public:
	void			updateChunkGridConfig(const NuiHashingChunkGridConfig& chunkGridConfig);
protected:
	void			rayIntervalSplatting(cl_mem cameraParamsCL, cl_mem transformCL);
	void			raycast(
		cl_mem renderVerticesCL,
		cl_mem renderNormalsCL,
		cl_mem renderIntensitiesCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		UINT nWidth, UINT nHeight
		);
private:
	void			AcquireBuffer();
	void			ReleaseBuffer();
	void			ResetVisibleEntrys();
	void			ResetAllocType();
	void			BuildHashAllocAndVisibleType(
		UINT nWidth, UINT nHeight,
		cl_mem floatDepthsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL);
	void			AllocateVoxelBlocksList();
private:
	NuiKinfuVoxelBlockHash		m_hashingVoxelData;
	NuiHashingOpenCLChunkGrid*	m_pChunkGrid;

	UINT						m_numOccupiedBlocks;

	NuiHashingRaycastConfig		m_raycastConfig;
	NuiMappable4f				m_raycastVertexBuffer;
	NuiTextureMappable			m_rayIntervalMinBuffer;
	NuiTextureMappable			m_rayIntervalMaxBuffer;


	// Reconstruction
	cl_mem		m_entriesAllocTypeCL;
	cl_mem		m_blockCoordsCL;

	// Render state
	/** A list of "visible entries", that are currently
			being processed by the tracker.
	*/
	cl_mem		m_visibleEntryIDsCL;
	/** A list of "visible entries", that are
			currently being processed by integration
			and tracker.
	*/
	cl_mem		m_entriesVisibleTypeCL;
	UINT		m_numVisibleEntries;

	NuiHashingSDFConfig	m_config;
};