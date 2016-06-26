#pragma once

#include "stdafx.h"
#include "NuiImageBuffer.h"

#include <vector>

struct NuiDevicePoint
{
	Point4D	fVertex;
	BGRQUAD fBGRA;
	bool	fIsValid;
	BYTE	fBodyIndex;
	float	fNormal_x;
	float	fNormal_y;
	float	fNormal_z;
	float	fColorSpaceU;
	float	fColorSpaceV;
};

class NuiDevicePointCloud
{
public:
	NuiDevicePointCloud();
	~NuiDevicePointCloud();

	void				DeepCopy (const NuiDevicePointCloud& other);
	NuiDevicePointCloud (const NuiDevicePointCloud& other){ DeepCopy(other); }
	NuiDevicePointCloud& operator = (const NuiDevicePointCloud& other) {	DeepCopy(other); return *this; }

	void				Clear();
	UINT				GetPointsNum() const { return m_nPointNum; }
	NuiDevicePoint*		AllocatePoints(UINT num);
	NuiDevicePoint*		AccessPoint(UINT id) const;
	bool				SetPoint(UINT id, const NuiDevicePoint& pt);
	bool				ReadPoint(UINT id, NuiDevicePoint* pPt) const;

	bool				SetPointVertex(UINT id, const Point4D& vt);

	void				SetWidthStep(UINT width) { m_nWidthStep = width; }
	UINT				GetWidthStep() const { return m_nWidthStep; }

	void				SetColorImage(const NuiColorImage& image) { m_colorImage = image; }
	const NuiColorImage&	GetColorImage() const { return m_colorImage; }
private:
	UINT				m_nPointNum;
	UINT				m_nWidthStep;
	NuiDevicePoint*		m_pCloudPoints;
	NuiColorImage		m_colorImage;
};