#pragma once

#include "stdafx.h"


#ifndef _FaceShapeAnimations_
#define _FaceShapeAnimations_
typedef enum _FaceShapeAnimations FaceShapeAnimations;


enum _FaceShapeAnimations
{
	FaceShapeAnimations_JawOpen	= 0,
	FaceShapeAnimations_LipPucker	= 1,
	FaceShapeAnimations_JawSlideRight	= 2,
	FaceShapeAnimations_LipStretcherRight	= 3,
	FaceShapeAnimations_LipStretcherLeft	= 4,
	FaceShapeAnimations_LipCornerPullerLeft	= 5,
	FaceShapeAnimations_LipCornerPullerRight	= 6,
	FaceShapeAnimations_LipCornerDepressorLeft	= 7,
	FaceShapeAnimations_LipCornerDepressorRight	= 8,
	FaceShapeAnimations_LeftcheekPuff	= 9,
	FaceShapeAnimations_RightcheekPuff	= 10,
	FaceShapeAnimations_LefteyeClosed	= 11,
	FaceShapeAnimations_RighteyeClosed	= 12,
	FaceShapeAnimations_RighteyebrowLowerer	= 13,
	FaceShapeAnimations_LefteyebrowLowerer	= 14,
	FaceShapeAnimations_LowerlipDepressorLeft	= 15,
	FaceShapeAnimations_LowerlipDepressorRight	= 16,
	FaceShapeAnimations_Count	= ( FaceShapeAnimations_LowerlipDepressorRight + 1 ) 
} ;
#endif // _FaceShapeAnimations_

class NuiFacialModel
{
public:
	NuiFacialModel();
	~NuiFacialModel();

	void				DeepCopy (const NuiFacialModel& other);
	NuiFacialModel (const NuiFacialModel& other){ DeepCopy(other); }
	NuiFacialModel& operator = (const NuiFacialModel& other) {	DeepCopy(other); return *this; }

	void Clear();
	CameraSpacePoint*	AllocateVertices(UINT nNum);

	float*				AllocateAUs(UINT nNum);
	float				GetAU(UINT i);

private:
	UINT				m_nVertexNum;
	CameraSpacePoint*	m_pVertices;
	UINT				m_nAUNum;
	float*				m_pAUs;
};