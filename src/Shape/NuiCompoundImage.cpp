#include "NuiCompoundImage.h"

NuiCompoundImage::NuiCompoundImage()
	: m_nWidth(0)
	, m_nHeight(0)
	, m_pPixels(NULL)
{
}


NuiCompoundImage::NuiCompoundImage(UINT nWidth, UINT nHeight)
	: m_nWidth(nWidth)
	, m_nHeight(nHeight)
{
	m_pPixels = new NuiCompoundPixel[m_nWidth*m_nHeight];
}

NuiCompoundImage::~NuiCompoundImage()
{
	Clear();
}

void NuiCompoundImage::Clear()
{
	SafeDeleteArray(m_pPixels);
	m_nWidth = 0;
	m_nHeight = 0;
}

void NuiCompoundImage::DeepCopy (const NuiCompoundImage& other)
{
	if(other.m_pPixels)
	{
		NuiCompoundPixel* pPixels = AllocatePixels(other.m_nWidth, other.m_nHeight);
		memcpy(pPixels, other.m_pPixels, m_nWidth*m_nHeight*sizeof(NuiCompoundPixel));
	}
	else
	{
		Clear();
	}
}

NuiCompoundPixel* NuiCompoundImage::AllocatePixels(UINT nWidth, UINT nHeight)
{
	if(m_pPixels)
	{
		if(m_nWidth*m_nHeight != nWidth*nHeight)
		{
			Clear();
		}
	}
	if(!m_pPixels)
	{
		m_nWidth = nWidth;
		m_nHeight = nWidth;
		if(0 != m_nWidth && 0 != m_nHeight)
		{
			m_pPixels = new NuiCompoundPixel[m_nWidth*m_nHeight];
		}
	}
	return m_pPixels;
}

bool NuiCompoundImage::SetPixel(UINT w, UINT h, const NuiCompoundPixel& pixel)
{
	if(!m_pPixels || w >= m_nWidth || h >= m_nHeight)
		return false;
	UINT index = h * m_nWidth + w;
	*(m_pPixels + index) = pixel;
	return true;
}

bool NuiCompoundImage::SetPixel(UINT index, const NuiCompoundPixel& pixel)
{
	if(!m_pPixels || index >= m_nWidth*m_nHeight)
		return false;
	*(m_pPixels + index) = pixel;
	return true;
}

NuiCompoundPixel* NuiCompoundImage::AccessPixel(UINT w, UINT h) const
{
	if(!m_pPixels || w >= m_nWidth || h >= m_nHeight)
		return NULL;

	UINT index = h * m_nWidth + w;
	return (m_pPixels + index);
}

bool NuiCompoundImage::ReadPixel(UINT w, UINT h, NuiCompoundPixel* pPixel) const
{
	if(!pPixel || !m_pPixels || w >= m_nWidth || h >= m_nHeight)
		return false;

	UINT index = h * m_nWidth + w;
	*pPixel = *(m_pPixels + index);
	return true;
}