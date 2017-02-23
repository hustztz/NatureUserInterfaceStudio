#pragma once

#include "Shape/NuiCameraPos.h"

class NuiKinfuCameraDeviceCacheImpl
{
public:
	virtual void UpdateCameraTransform(const Matrix3frm& rot, const Vector3f& tran) = 0;
	virtual void UpdateCameraParams(const NuiCameraParams& cameraParams, UINT nWidth, UINT nHeight) = 0;
};

class NuiKinfuCameraState
{
public:
	NuiKinfuCameraState(NuiKinfuCameraDeviceCacheImpl* device) : m_deviceCache(device){}
	~NuiKinfuCameraState() { SafeDelete(m_deviceCache); }

	void UpdateCameraTransform(const Matrix3frm& rot, const Vector3f& tran)
	{
		m_pos.setRotation(rot);
		m_pos.setTranslation(tran);
		if(m_deviceCache)
			m_deviceCache->UpdateCameraTransform(rot, tran);
	}
	void UpdateCameraParams(const NuiCameraParams& cameraParams, UINT nWidth, UINT nHeight)
	{
		m_pos.setIntrinsics(cameraParams.m_intrinsics);
		m_pos.setSensorDepthRange(cameraParams.m_sensorDepthMin, cameraParams.m_sensorDepthMax);
		if(m_deviceCache)
			m_deviceCache->UpdateCameraParams(cameraParams, nWidth, nHeight);
	}
	const NuiCameraPos& GetCameraPos() const { return m_pos; }
	NuiKinfuCameraDeviceCacheImpl* GetDeviceCache() const { return m_deviceCache; }
	
private:
	NuiCameraPos					m_pos;
	NuiKinfuCameraDeviceCacheImpl*	m_deviceCache;
};