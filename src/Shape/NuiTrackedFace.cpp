#include "NuiTrackedFace.h"
#define _USE_MATH_DEFINES
#include <math.h>

bool NuiTrackedFace::SetFace2DPoint(UINT pointType, PointF point)
{
	if(pointType >= FacePointType_Count)
		return false;

	m_face2DPoints[pointType] = point;
	return true;
}

bool NuiTrackedFace::GetFace2DPoint(UINT pointType, PointF* pPoint) const
{
	if(!pPoint || (pointType >= FacePointType_Count))
		return false;

	*pPoint = m_face2DPoints[pointType];
	return true;
}


bool NuiTrackedFace::SetFaceProperties(UINT propertyType, DetectionResult property)
{
	if(propertyType >= FaceProperty_Count)
		return false;

	m_faceProperties[propertyType] = property;
	return true;
}

bool NuiTrackedFace::GetFaceProperties(UINT propertyType, DetectionResult* pProperty) const
{
	if(!pProperty || (propertyType >= FaceProperty_Count))
		return false;

	*pProperty = m_faceProperties[propertyType];
	return true;
}

/*static*/
void NuiTrackedFace::ExtractFaceRotationInDegrees( const Vector4* pQuaternion, double* pPitch, double* pYaw, double* pRoll )
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;

	// convert face rotation quaternion to Euler angles in degrees
	*pPitch = atan2( 2 * ( y * z + w * x ), w * w - x * x - y * y + z * z ) / M_PI * 180.0f;
	*pYaw = asin( 2 * ( w * y - x * z ) ) / M_PI * 180.0f;
	*pRoll = atan2( 2 * ( x * y + w * z ), w * w + x * x - y * y - z * z ) / M_PI * 180.0f;
}

void NuiTrackedFace::GetRotationXYZ(double* pRotationX, double* pRotationY, double* pRotationZ) const
{
	if(!pRotationX || !pRotationY || !pRotationZ)
		return;

	ExtractFaceRotationInDegrees(&m_rotationQuaternion, pRotationX, pRotationY, pRotationZ);
}
