#pragma once

#include "stdafx.h"

class NuiFacialModel
{
public:
	NuiFacialModel();
	~NuiFacialModel();

	void				DeepCopy (const NuiFacialModel& other);
	NuiFacialModel (const NuiFacialModel& other){ DeepCopy(other); }
	NuiFacialModel& operator = (const NuiFacialModel& other) {	DeepCopy(other); return *this; }

	void Clear();
	CameraSpacePoint* AllocateVertices(UINT nNum);

private:
	UINT				m_nVertexNum;
	CameraSpacePoint*	m_pVertices;
};