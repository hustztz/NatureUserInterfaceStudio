#pragma once

#include "Foundation/NuiFileIOUtilities.h"

struct NuiCameraIntrinsics
{
	float m_fx;
	float m_fy;
	float m_cx;
	float m_cy;

	NuiCameraIntrinsics()
		: m_cx(0.0f)
		, m_cy(0.0f)
		, m_fx(0.0f)
		, m_fy(0.0f)
	{
	}
	NuiCameraIntrinsics(float fx, float fy, float cx, float cy)
		: m_fx(fx)
		, m_fy(fy)
		, m_cx(cx)
		, m_cy(cy)
	{
	}


};

struct NuiCameraParams
{
	NuiCameraIntrinsics		m_intrinsics;

	float m_sensorDepthMin;
	float m_sensorDepthMax;


	bool save(const std::string& fileName)
	{
		return NuiFileIOUtilities::writeCamera(fileName, m_intrinsics.m_fx, m_intrinsics.m_fy, m_intrinsics.m_cx, m_intrinsics.m_cy);
	}

	bool load(const std::string& fileName)
	{
		return NuiFileIOUtilities::readCamera(fileName, &m_intrinsics.m_fx, &m_intrinsics.m_fy, &m_intrinsics.m_cx, &m_intrinsics.m_cy);
	}
};