#pragma once

#include "NuiKinfuVolume.h"
#include "NuiHashingSDFConfig.h"
#include "NuiHashingRaycastConfig.h"
#include "NuiHashingChunkGridConfig.h"
#include "Foundation/SgVec3T.h"

#include "OpenCLUtilities/NuiMappable.h"
#include "OpenCLUtilities/NuiTextureMappable.h"

// Forwards
class NuiHashingSDF;
class NuiHashingChunkGrid;

class NuiHashingVolume : public NuiKinfuVolume
{
public:
	NuiHashingVolume(const NuiHashingSDFConfig& sdfConfig, const NuiHashingRaycastConfig& raycastConfig);
	~NuiHashingVolume();

	virtual void	reset() override;
	virtual bool	log(const std::string& fileName) const override;

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
		cl_mem renderIntensitiesCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		UINT nWidth, UINT nHeight,
		float sensorDepthMin, float sensorDepthMax
		) override;

	virtual void	offlineRender() override;

	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) override;

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
	NuiHashingSDF*				m_pSDFData;
	NuiHashingChunkGrid*		m_pChunkGrid;

	UINT						m_numOccupiedBlocks;

	NuiHashingRaycastConfig		m_raycastConfig;
	NuiMappable4f				m_raycastVertexBuffer;
	NuiTextureMappable			m_rayIntervalMinBuffer;
	NuiTextureMappable			m_rayIntervalMaxBuffer;

	std::atomic<bool>			m_offlineRenderDirty;

	// For offline render
	cl_mem m_renderVerticesCL;
	cl_mem m_renderNormalsCL;
	cl_mem m_renderIntensitiesCL;
	cl_mem m_cameraParamsCL;
	cl_mem m_transformCL;
	UINT m_nWidth; UINT m_nHeight;
	float m_sensorDepthMin; float m_sensorDepthMax;
};