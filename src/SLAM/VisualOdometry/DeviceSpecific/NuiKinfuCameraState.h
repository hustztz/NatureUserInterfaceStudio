#pragma once

#include "Shape/NuiCameraPos.h"

class NuiKinfuCameraState
{
public:
	NuiKinfuCameraState(){}
	virtual ~NuiKinfuCameraState() {}

	virtual void UpdateCameraTransform(const Matrix3frm& rot, const Vector3f& tran)
	{
		m_pos.setRotation(rot);
		m_pos.setTranslation(tran);
	}
	virtual void UpdateCameraParams(const NuiCameraParams& cameraParams, UINT nWidth, UINT nHeight)
	{
		m_pos.setIntrinsics(cameraParams.m_intrinsics);
		m_pos.setSensorDepthRange(cameraParams.m_sensorDepthMin, cameraParams.m_sensorDepthMax);
	}
	const NuiCameraPos& GetCameraPos() const { return m_pos; }
	
private:
	NuiCameraPos					m_pos;
};