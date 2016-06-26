#pragma once

/** Available joints in skeleton */
typedef enum
{
	JOINT_TYPE_HIP_CENTER	= 0,
	JOINT_TYPE_SPINE	= ( JOINT_TYPE_HIP_CENTER + 1 ) ,
	JOINT_TYPE_NECK	= ( JOINT_TYPE_SPINE + 1 ) ,
	JOINT_TYPE_HEAD	= ( JOINT_TYPE_NECK + 1 ) ,
	JOINT_TYPE_SHOULDER_LEFT	= ( JOINT_TYPE_HEAD + 1 ) ,
	JOINT_TYPE_ELBOW_LEFT	= ( JOINT_TYPE_SHOULDER_LEFT + 1 ) ,
	JOINT_TYPE_WRIST_LEFT	= ( JOINT_TYPE_ELBOW_LEFT + 1 ) ,
	JOINT_TYPE_HAND_LEFT	= ( JOINT_TYPE_WRIST_LEFT + 1 ) ,
	JOINT_TYPE_SHOULDER_RIGHT	= ( JOINT_TYPE_HAND_LEFT + 1 ) ,
	JOINT_TYPE_ELBOW_RIGHT	= ( JOINT_TYPE_SHOULDER_RIGHT + 1 ) ,
	JOINT_TYPE_WRIST_RIGHT	= ( JOINT_TYPE_ELBOW_RIGHT + 1 ) ,
	JOINT_TYPE_HAND_RIGHT	= ( JOINT_TYPE_WRIST_RIGHT + 1 ) ,
	JOINT_TYPE_HIP_LEFT	= ( JOINT_TYPE_HAND_RIGHT + 1 ) ,
	JOINT_TYPE_KNEE_LEFT	= ( JOINT_TYPE_HIP_LEFT + 1 ) ,
	JOINT_TYPE_ANKLE_LEFT	= ( JOINT_TYPE_KNEE_LEFT + 1 ) ,
	JOINT_TYPE_FOOT_LEFT	= ( JOINT_TYPE_ANKLE_LEFT + 1 ) ,
	JOINT_TYPE_HIP_RIGHT	= ( JOINT_TYPE_FOOT_LEFT + 1 ) ,
	JOINT_TYPE_KNEE_RIGHT	= ( JOINT_TYPE_HIP_RIGHT + 1 ) ,
	JOINT_TYPE_ANKLE_RIGHT	= ( JOINT_TYPE_KNEE_RIGHT + 1 ) ,
	JOINT_TYPE_FOOT_RIGHT	= ( JOINT_TYPE_ANKLE_RIGHT + 1 ) ,
	JOINT_TYPE_SHOULDER_CENTER	= ( JOINT_TYPE_FOOT_RIGHT + 1 ) ,
	JOINT_TYPE_HAND_TIP_LEFT	= ( JOINT_TYPE_SHOULDER_CENTER + 1 ) ,
	JOINT_TYPE_THUMB_LEFT	= ( JOINT_TYPE_HAND_TIP_LEFT + 1 ) ,
	JOINT_TYPE_HAND_TIP_RIGHT	= ( JOINT_TYPE_THUMB_LEFT + 1 ) ,
	JOINT_TYPE_THUMB_RIGHT	= ( JOINT_TYPE_HAND_TIP_RIGHT + 1 ) ,
	JOINT_TYPE_COUNT	= ( JOINT_TYPE_THUMB_RIGHT + 1 )
} NuiJointType;


#ifndef _HandState_
#define _HandState_
typedef enum _HandState HandState;
enum _HandState
{
	HandState_Unknown	= 0,
	HandState_NotTracked	= 1,
	HandState_Open	= 2,
	HandState_Closed	= 3,
	HandState_Lasso	= 4
} ;
#endif // _HandState_


struct NuiJoint3DPos
{
	float x;
	float y;
	float z;

	bool isInferred;
};

struct NuiJoint2DPos
{
	float x;
	float y;

	bool isInferred;
};

class NuiSkeletonJoints
{
public:
	NuiSkeletonJoints();
	~NuiSkeletonJoints() {}

	/*void		DeepCopy (const NuiSkeletonJoints& other);
	NuiSkeletonJoints (const NuiSkeletonJoints& other) { DeepCopy(other); }
	NuiSkeletonJoints& operator=( const NuiSkeletonJoints& other )  {	DeepCopy(other); return *this; }*/

	void	SetInvalid();

	void	SetJoint3DPos(NuiJointType jointType, float x, float y, float z, bool isInferred = false);
	void	SetJoint2DPos(NuiJointType jointType, float x, float y, bool isInferred = false);
	void	SetLeftHand(HandState state) { m_leftHand = state; }
	void	SetRightHand(HandState state) { m_rightHand = state; }

	bool	IsInferred(NuiJointType jointType) const;
	float	GetJoint3DPosX(NuiJointType jointType) const;
	float	GetJoint3DPosY(NuiJointType jointType) const;
	float	GetJoint3DPosZ(NuiJointType jointType) const;

	float	GetJoint2DPosX(NuiJointType jointType) const;
	float	GetJoint2DPosY(NuiJointType jointType) const;

	HandState	GetLeftHand() const { return m_leftHand; }
	HandState	GetRightHand() const { return m_rightHand; }

	void	SetHeadForFaceTracking(float x, float y, float z, bool isInferred = false);
	void	SetNeckForFaceTracking(float x, float y, float z, bool isInferred = false);
	bool	IsHeadInferred() const { return m_headForFaceTracking.isInferred; }
	float	GetHeadPosX() const  { return m_headForFaceTracking.x; }
	float	GetHeadPosY() const  { return m_headForFaceTracking.y; }
	float	GetHeadPosZ() const  { return m_headForFaceTracking.z; }
	bool	IsNeckInferred() const { return m_neckForFaceTracking.isInferred; }
	float	GetNeckPosX() const { return m_neckForFaceTracking.x; }
	float	GetNeckPosY() const { return m_neckForFaceTracking.y; }
	float	GetNeckPosZ() const { return m_neckForFaceTracking.z; }

private:
	NuiJoint3DPos		m_joints[JOINT_TYPE_COUNT];
	NuiJoint2DPos		m_joints2D[JOINT_TYPE_COUNT];
	HandState			m_leftHand;
	HandState			m_rightHand;

	// For KinectV1 only
	NuiJoint3DPos		m_headForFaceTracking;
	NuiJoint3DPos		m_neckForFaceTracking;
};