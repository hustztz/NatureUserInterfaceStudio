#include "NuiSkeletonJoints.h"

NuiSkeletonJoints::NuiSkeletonJoints()
{
	SetInvalid();
}

void NuiSkeletonJoints::SetInvalid()
{
	for (int i = 0; i < JOINT_TYPE_COUNT; ++i)
	{
		m_joints[i].isInferred = true;
	}
	for (int i = 0; i < JOINT_TYPE_COUNT; ++i)
	{
		m_joints2D[i].isInferred = true;
	}
	m_leftHand = HandState_Unknown;
	m_rightHand = HandState_Unknown;
	m_headForFaceTracking.isInferred = true;
	m_neckForFaceTracking.isInferred = true;
}

void NuiSkeletonJoints::SetJoint3DPos(NuiJointType jointType, float x, float y, float z, bool isInferred /*= false*/)
{
	if(jointType >= JOINT_TYPE_COUNT)
		return;
	m_joints[jointType].x = x;
	m_joints[jointType].y = y;
	m_joints[jointType].z = z;
	m_joints[jointType].isInferred = isInferred;
}

void NuiSkeletonJoints::SetJoint2DPos(NuiJointType jointType, float x, float y, bool isInferred /*= false*/)
{
	if(jointType >= JOINT_TYPE_COUNT)
		return;
	m_joints2D[jointType].x = x;
	m_joints2D[jointType].y = y;
	m_joints2D[jointType].isInferred = isInferred;
}

bool	NuiSkeletonJoints::IsInferred(NuiJointType jointType) const
{
	if(jointType >= JOINT_TYPE_COUNT)
		return false;
	return m_joints[jointType].isInferred;
}

float	NuiSkeletonJoints::GetJoint3DPosX(NuiJointType jointType) const
{
	if(jointType >= JOINT_TYPE_COUNT)
		return 0.0f;
	return m_joints[jointType].x;
}

float	NuiSkeletonJoints::GetJoint3DPosY(NuiJointType jointType) const
{
	if(jointType >= JOINT_TYPE_COUNT)
		return 0.0f;
	return m_joints[jointType].y;
}

float	NuiSkeletonJoints::GetJoint3DPosZ(NuiJointType jointType) const
{
	if(jointType >= JOINT_TYPE_COUNT)
		return 0.0f;
	return m_joints[jointType].z;
}

float	NuiSkeletonJoints::GetJoint2DPosX(NuiJointType jointType) const
{
	if(jointType >= JOINT_TYPE_COUNT)
		return 0.0f;
	return m_joints2D[jointType].x;
}

float	NuiSkeletonJoints::GetJoint2DPosY(NuiJointType jointType) const
{
	if(jointType >= JOINT_TYPE_COUNT)
		return 0.0f;
	return m_joints2D[jointType].y;
}

void	NuiSkeletonJoints::SetHeadForFaceTracking(float x, float y, float z, bool isInferred /*= false*/)
{
	m_headForFaceTracking.isInferred = isInferred;
	m_headForFaceTracking.x = x;
	m_headForFaceTracking.y = y;
	m_headForFaceTracking.z = z;
}

void	NuiSkeletonJoints::SetNeckForFaceTracking(float x, float y, float z, bool isInferred /*= false*/)
{
	m_neckForFaceTracking.isInferred = isInferred;
	m_neckForFaceTracking.x = x;
	m_neckForFaceTracking.y = y;
	m_neckForFaceTracking.z = z;
}

//void NuiSkeletonJoints::DeepCopy( const NuiSkeletonJoints& other )
//{
//	if ( &other != this ) {
//		for (int i = 0; i < JOINT_TYPE_COUNT; ++i)
//		{
//			m_joints[i]      = other.m_joints[i];
//		}
//		for (int i = 0; i < JOINT_TYPE_COUNT; ++i)
//		{
//			m_joints2D[i]      = other.m_joints2D[i];
//		}
//	}
//}