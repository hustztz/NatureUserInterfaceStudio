#pragma once

#include "stdafx.h"

class NuiPointCloudImpl
{
public:
	NuiPointCloudImpl();
	virtual ~NuiPointCloudImpl();

	virtual void		Clear() = 0;
	virtual UINT		GetPointsNum() const = 0;
	virtual bool		SetPointVertex(UINT id, const Point4D& vt) = 0;
	virtual bool		ReadPointVertex(UINT id, Point4D* pVt) = 0;
};