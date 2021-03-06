#pragma once

#include "../NuiKinfuScene.h"
#include "../../NuiHashingSDFConfig.h"
#include "../../NuiHashingRaycastConfig.h"
#include "../../NuiHashingChunkGridConfig.h"
#include "Foundation/SgVec3T.h"

#include "OpenCLUtilities/NuiOpenCLUtil.h"
#include "OpenCLUtilities/NuiMappable.h"
#include "OpenCLUtilities/NuiTextureMappable.h"

// Forwards
class NuiHashingOpenCLSDF;
class NuiHashingOpenCLChunkGrid;

class NuiHashingOpenCLScene : public NuiKinfuScene
{
public:
	NuiHashingOpenCLScene(const NuiHashingSDFConfig& sdfConfig, const NuiHashingRaycastConfig& raycastConfig);
	~NuiHashingOpenCLScene();

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
	
	virtual void	offlineRender() override;

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
private:
	NuiHashingOpenCLSDF*		m_pSDFData;
	NuiHashingOpenCLChunkGrid*	m_pChunkGrid;

	UINT						m_numOccupiedBlocks;

	NuiHashingRaycastConfig		m_raycastConfig;
	NuiMappable4f				m_raycastVertexBuffer;
	NuiTextureMappable			m_rayIntervalMinBuffer;
	NuiTextureMappable			m_rayIntervalMaxBuffer;

	std::atomic<bool>			m_offlineRenderDirty;

	cl_mem						m_vertexSumCL;

	// For offline render
	cl_mem m_renderVerticesCL;
	cl_mem m_renderNormalsCL;
	cl_mem m_renderIntensitiesCL;
	cl_mem m_cameraParamsCL;
	cl_mem m_transformCL;
	UINT m_nWidth; UINT m_nHeight;
	float m_sensorDepthMin; float m_sensorDepthMax;
};