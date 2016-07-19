#pragma once

#include "NuiKinfuVolume.h"
#include "NuiHashingSDF.h"

#include "Foundation/SgVec3T.h"

// Forwards
class NuiHashingChunkGrid;

class NuiHashingVolume : public NuiKinfuVolume
{
public:
	NuiHashingVolume();
	~NuiHashingVolume();

	virtual void	reset() override;

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
		cl_mem cameraParamsCL,
		const NuiKinfuTransform& currPos,
		UINT nWidth, UINT nHeight
		) override;

	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) override;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) override;

protected:
	void	raycastRender(
		NuiHashingSDF* pSDF,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		cl_mem verticesCL,
		cl_mem normalsCL,
		float rayIncrement,
		float thresSampleDist,
		float thresDist,
		float minDepth,
		float maxDepth,
		UINT nWidth, UINT nHeight);
private:
	NuiHashingSDF			m_sdfData;
	NuiHashingChunkGrid*	m_pChunkGrid;

	/*Matrix3frm				m_lastIntegrationRotation;
	Vector3f				m_lastIntegrationTranslation;*/

	// Raycast Params
	float					m_rayIncrement;
	float					m_thresSampleDist;
	float					m_thresDist;
	float					m_minDepth;
	float					m_maxDepth;
};