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

	virtual void	incrementVolume(
		cl_mem floatDepthsCL,
		cl_mem colorsCL,
		cl_mem normalsCL,
		cl_mem cameraParamsCL,
		const NuiKinfuTransform& currPos,
		UINT nWidth, UINT nHeight
		) override;
	virtual bool	evaluateVolume(
		cl_mem floatDepthsCL,
		cl_mem colorsCL,
		cl_mem normalsCL,
		cl_mem renderVertices,
		cl_mem renderNormals,
		cl_mem renderColors,
		cl_mem cameraParamsCL,
		const NuiKinfuTransform& currPos,
		UINT nWidth, UINT nHeight
		) override;

	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) override;

	void			updateChunkGridConfig(const NuiHashingChunkGridConfig& chunkGridConfig);
protected:
	void	raycastRender(
		NuiHashingSDF* pSDF,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		cl_mem verticesCL,
		cl_mem normalsCL,
		cl_mem colorsCL,
		float rayIncrement,
		float thresSampleDist,
		float thresDist,
		UINT nWidth, UINT nHeight
		);
private:
	NuiHashingSDF*			m_pSDFData;
	NuiHashingChunkGrid*	m_pChunkGrid;

	NuiHashingRaycastConfig		m_raycastConfig;
};