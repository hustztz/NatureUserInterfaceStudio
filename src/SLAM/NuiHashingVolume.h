#pragma once

#include "NuiKinfuVolume.h"
#include "NuiHashingSDFConfig.h"
#include "NuiHashingRaycastConfig.h"
#include "NuiHashingChunkGridConfig.h"
#include "Foundation/SgVec3T.h"

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
		UINT nWidth, UINT nHeight
		) override;

	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) override;

	void			updateChunkGridConfig(const NuiHashingChunkGridConfig& chunkGridConfig);
private:
	NuiHashingSDF*			m_pSDFData;
	NuiHashingChunkGrid*	m_pChunkGrid;

	NuiHashingRaycastConfig		m_raycastConfig;
};