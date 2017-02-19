#include "NuiFacialModel.h"

NuiFacialModel::NuiFacialModel()
	: m_nVertexNum(0)
	, m_pVertices(NULL)
	, m_nAUNum(0)
	, m_pAUs(NULL)
{}

NuiFacialModel::~NuiFacialModel()
{
	Clear();
}

void NuiFacialModel::Clear()
{
	SafeDeleteArray(m_pVertices);
	m_nVertexNum = 0;

	SafeDeleteArray(m_pAUs);
	m_nAUNum = 0;
}

void NuiFacialModel::DeepCopy (const NuiFacialModel& other)
{
	if(other.m_pVertices)
	{
		CameraSpacePoint* pVertices = AllocateVertices(other.m_nVertexNum);
		memcpy(pVertices, other.m_pVertices, m_nVertexNum * sizeof(CameraSpacePoint));
	}
	else
	{
		SafeDeleteArray(m_pVertices);
		m_nVertexNum = 0;
	}

	if(other.m_nAUNum)
	{
		float* pAUs = AllocateAUs(other.m_nAUNum);
		memcpy(pAUs, other.m_pAUs, m_nAUNum * sizeof(float));
	}
	else
	{
		SafeDeleteArray(m_pAUs);
		m_nAUNum = 0;
	}
}

CameraSpacePoint* NuiFacialModel::AllocateVertices(UINT nNum)
{
	if(m_pVertices)
	{
		if(m_nVertexNum != nNum)
		{
			SafeDeleteArray(m_pVertices);
			m_nVertexNum = 0;
		}
	}
	if(!m_pVertices)
	{
		m_nVertexNum = nNum;
		if(m_nVertexNum > 0)
		{
			m_pVertices = new CameraSpacePoint[m_nVertexNum];
		}
	}
	return m_pVertices;
}

float*		NuiFacialModel::AllocateAUs(UINT nNum)
{
	if(m_pAUs)
	{
		if(m_nAUNum != nNum)
		{
			SafeDeleteArray(m_pAUs);
			m_pAUs = 0;
		}
	}
	if(!m_pAUs)
	{
		m_nAUNum = nNum;
		if(m_nAUNum > 0)
		{
			m_pAUs = new float[m_nAUNum];
		}
	}
	return m_pAUs;
}

float			NuiFacialModel::GetAU(UINT i)
{
	if(!m_pAUs || i >= m_nAUNum)
		return 0.0;

	return m_pAUs[i];
}