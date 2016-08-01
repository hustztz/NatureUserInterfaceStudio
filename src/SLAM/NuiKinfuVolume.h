#pragma once

#include "stdafx.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <atomic>

//Forwards
class NuiKinfuTransform;
class NuiCLMappableData;
class NuiMeshShape;

class NuiKinfuVolume
{
public:
	NuiKinfuVolume();
	~NuiKinfuVolume();

	void setDirty() { m_dirty = true; }
	void clearDirty() { m_dirty = false; }

	virtual bool	log(const std::string& fileName) const = 0;
	virtual bool	hasColorData() const { return true; }
	/** \brief Resets tsdf volume data to uninitialized state */
	virtual void	reset() = 0;
	virtual void	integrateVolume(
		cl_mem floatDepthsCL,
		cl_mem normalsCL,
		cl_mem colorsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		UINT nWidth, UINT nHeight
		) = 0;
	virtual void    raycastRender(
		cl_mem renderVerticesCL,
		cl_mem renderNormalsCL,
		cl_mem renderColorsCL,
		cl_mem cameraParamsCL,
		cl_mem transformCL,
		UINT nWidth, UINT nHeight
		) = 0;

	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) = 0;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) = 0;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) = 0;

protected:
	void			AcquireBuffer(bool bHas_color_volume);
	void			ReleaseBuffer();

protected:
	// Buffers for output
	cl_mem				m_volumeOutputVerticesCL;
	cl_mem				m_volumeOutputColorsCL;
	cl_mem				m_vertexSumCL;

	std::atomic<bool>	m_dirty;
};