#pragma once

#include "../NuiKinfuScene.h"
#include "NuiKinfuVoxelBlockHash.h"
#include "NuiOpenCLPrefixSum.h"

#include "../../NuiHashingSDFConfig.h"

// Forwards
class NuiKinfuOpenCLHashGlobalCache;

class NuiKinfuOpenCLHashScene : public NuiKinfuScene
{
public:
	NuiKinfuOpenCLHashScene(const NuiHashingSDFConfig& sdfConfig);
	~NuiKinfuOpenCLHashScene();

	virtual void	reset() override;
	virtual bool	log(const std::string& fileName) const override;

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

public:
	void			updateGlobalCacheConfig(const NuiHashingChunkGridConfig& chunkGridConfig);

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

	void			AllocateSceneFromDepth(
		UINT nWidth, UINT nHeight,
		cl_mem floatDepthsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL);
	void			IntegrateIntoScene(
		cl_mem floatDepthsCL,
		cl_mem colorsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL);

private:
	void			AcquireBuffers();
	void			ReleaseBuffers();
	void			ResetVisibleEntrys();
	void			ResetAllocType();
	void			BuildHashAllocAndVisibleType(
		UINT nWidth, UINT nHeight,
		cl_mem floatDepthsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL);
	void			AllocateVoxelBlocksList();
	void			BuildVisibleList(cl_mem cameraParamsCL, cl_mem transformCL);
	void			ReAllocateSwappedOutVoxelBlocks();

	void			CreateExpectedDepths(cl_mem expectedRangeCL, cl_mem cameraParamsCL, cl_mem transformCL);
	void			raycast(
		cl_mem renderVerticesCL,
		cl_mem renderNormalsCL,
		cl_mem renderColorsCL,
		cl_mem expectedRangeCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		UINT nWidth, UINT nHeight
		);

private:
	NuiKinfuVoxelBlockHash			m_hashingVoxelData;
	NuiKinfuOpenCLHashGlobalCache*	m_pGlobalCache;

	NuiOpenCLPrefixSum				m_scan;
	
	NuiHashingSDFConfig				m_config;
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
	cl_mem		m_entriesVisibleTypePrefixCL;
	UINT		m_numVisibleEntries;

	// Fetch scene
	cl_mem		m_outputIdxCL;
};