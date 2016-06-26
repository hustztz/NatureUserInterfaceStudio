#include "NuiFacialModel.h"

NuiFacialModel::NuiFacialModel()
	: m_nVertexNum(0)
	, m_pVertices(NULL)
{}

NuiFacialModel::~NuiFacialModel()
{
	Clear();
}

void NuiFacialModel::Clear()
{
	SafeDeleteArray(m_pVertices);
	m_nVertexNum = 0;
}

void NuiFacialModel::DeepCopy (const NuiFacialModel& other)
{
	if(other.m_pVertices)
	{
		CameraSpacePoint* pBuffer = AllocateVertices(other.m_nVertexNum);
		memcpy(pBuffer, other.m_pVertices, m_nVertexNum * sizeof(CameraSpacePoint));
	}
	else
	{
		Clear();
	}
}

CameraSpacePoint* NuiFacialModel::AllocateVertices(UINT nNum)
{
	if(m_pVertices)
	{
		if(m_nVertexNum != nNum)
		{
			Clear();
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