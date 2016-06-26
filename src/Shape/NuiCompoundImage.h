#pragma once

#include "stdafx.h"

struct NuiCompoundPixel
{
	UINT16	fDepth;
	BGRQUAD fColor;
	BYTE	fBodyIndex;
};

class NuiCompoundImage
{
public:
	NuiCompoundImage();
	NuiCompoundImage(UINT nWidth, UINT nHeight);
	~NuiCompoundImage();

	void		Clear();
	void		DeepCopy (const NuiCompoundImage& other);
	NuiCompoundImage (const NuiCompoundImage& other){ DeepCopy(other); }
	NuiCompoundImage& operator = (const NuiCompoundImage& other) {	DeepCopy(other); return *this; }

	NuiCompoundPixel* AllocatePixels(UINT nWidth, UINT nHeight);
	NuiCompoundPixel* Data() const { return m_pPixels; }
	UINT	GetWidth() const { return m_nWidth; }
	UINT	GetHeight() const { return m_nHeight; }
	bool SetPixel(UINT w, UINT h, const NuiCompoundPixel& pixel);
	bool SetPixel(UINT index, const NuiCompoundPixel& pixel);
	NuiCompoundPixel* AccessPixel(UINT w, UINT h) const;
	bool ReadPixel(UINT w, UINT h, NuiCompoundPixel* pPixel) const;

private:
	UINT				m_nWidth;
	UINT				m_nHeight;
	NuiCompoundPixel*	m_pPixels;
};