#pragma once

#include "stdafx.h"

#include <string>
#include <atomic>

//Forwards
class NuiKinfuFrame;
class NuiKinfuFeedbackFrame;
class NuiKinfuCameraState;
class NuiKinfuTracker;
class NuiCLMappableData;
class NuiMeshShape;

class NuiKinfuScene
{
public:
	NuiKinfuScene(){}
	virtual ~NuiKinfuScene(){}

	void setDirty() { m_dirty = true; }
	void clearDirty() { m_dirty = false; }

	virtual bool	log(const std::string& fileName) const = 0;
	virtual bool	hasColorData() const { return true; }
	virtual bool	needAcceleratedFeedback() const { return false; }
	/** \brief Resets tsdf volume data to uninitialized state */
	virtual void	reset() = 0;
	virtual bool	integrateVolume(
		NuiKinfuFrame*			pFrame,
		NuiKinfuCameraState*	pCameraState
		) = 0;
	virtual void	raycastRender(
		NuiKinfuFeedbackFrame*	pFeedbackFrame,
		NuiKinfuCameraState*	pCameraState
		) = 0;
	virtual void	offlineRender() {};

	virtual bool	Volume2CLVertices(NuiCLMappableData* pCLData) = 0;
	virtual bool	Volume2CLMesh(NuiCLMappableData* pCLData) = 0;
	virtual bool	Volume2Mesh(NuiMeshShape* pMesh) = 0;

protected:
	std::atomic<bool>	m_dirty;
};