#pragma once

#include "stdafx.h"

#ifndef _FacePointType_
#define _FacePointType_
typedef enum _FacePointType FacePointType;
enum _FacePointType
{
	FacePointType_None	= -1,
	FacePointType_EyeLeft	= 0,
	FacePointType_EyeRight	= 1,
	FacePointType_Nose	= 2,
	FacePointType_MouthCornerLeft	= 3,
	FacePointType_MouthCornerRight	= 4,
	FacePointType_Count	= ( FacePointType_MouthCornerRight + 1 ) 
} ;
#endif // _FacePointType_

#ifndef _FaceProperty_
#define _FaceProperty_
typedef enum _FaceProperty FaceProperty;
enum _FaceProperty
{
	FaceProperty_Happy	= 0,
	FaceProperty_Engaged	= 1,
	FaceProperty_WearingGlasses	= 2,
	FaceProperty_LeftEyeClosed	= 3,
	FaceProperty_RightEyeClosed	= 4,
	FaceProperty_MouthOpen	= 5,
	FaceProperty_MouthMoved	= 6,
	FaceProperty_LookingAway	= 7,
	FaceProperty_Count	= ( FaceProperty_LookingAway + 1 ) 
} ;
#endif // _FaceProperty_

class NuiTrackedFace
{
public:
	NuiTrackedFace() {}
	~NuiTrackedFace() {}

	void	SetRotationQuaternion(const Vector4& data) { m_rotationQuaternion = data; }
	void	GetRotationXYZ(double* pRotationX, double* pRotationY, double* pRotationZ) const;

	void	SetBoundingBox(const RectI& data) { m_boundingBox = data; }
	const RectI& GetBoundingBox() const { return m_boundingBox; }

	bool	SetFace2DPoint(UINT pointType, PointF point);
	bool	GetFace2DPoint(UINT pointType, PointF* pPoint) const;

	bool	SetFaceProperties(UINT propertyType, DetectionResult property);
	bool	GetFaceProperties(UINT propertyType, DetectionResult* pProperty) const;

protected:
	// Quote from Kinect for Windows SDK v2.0 Developer Preview - Samples/Native/FaceBasics-D2D, and Partial Modification
	// ExtractFaceRotationInDegrees is: Copyright (c) Microsoft Corporation. All rights reserved.
	static void ExtractFaceRotationInDegrees( const Vector4* pQuaternion, double* pPitch, double* pYaw, double* pRoll );

private:
	Vector4			m_rotationQuaternion;
	RectI			m_boundingBox;
	PointF			m_face2DPoints[FacePointType::FacePointType_Count];
	DetectionResult	m_faceProperties[FaceProperty::FaceProperty_Count];
};