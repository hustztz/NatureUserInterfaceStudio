#pragma once

#include "NuiGrabberFrame.h"
#include "Shape/NuiFacialModel.h"

class NuiFacialModelFrame : public NuiGrabberFrame
{
public:
	NuiFacialModelFrame();
	virtual ~NuiFacialModelFrame();

	virtual void		Clear() override;
	bool				CacheFacialModel(UINT faceId, NuiFacialModel* pFace);
	bool				ReadFacialModel(UINT faceId, NuiFacialModel* pFace) const;

private:
	// Disable
	NuiFacialModelFrame (const NuiFacialModelFrame& other);
	NuiFacialModelFrame& operator = (const NuiFacialModelFrame& other);
private:
	NuiFacialModel*		m_facialModel[sBodyCount];
};