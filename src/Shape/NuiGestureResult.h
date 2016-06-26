#pragma once

#include "stdafx.h"

typedef enum
{
	Gesture_Discrete_None = -1,
	Gesture_Discrete_HandUp = ( Gesture_Discrete_None + 1),
	Gesture_Discrete_OpenHand = ( Gesture_Discrete_HandUp + 1),

	Gesture_Discrete_COUNT = ( Gesture_Discrete_OpenHand + 1)
} NuiDiscreteGestureType;

struct NuiDiscreteGestureResult
{
	bool bDetected;
	float fConfidence;

	NuiDiscreteGestureResult()
		: bDetected(false)
		, fConfidence(0.0f)
	{};
};

typedef enum 
{
	Gesture_Continuous_None = -1,
	Gesture_Continuous_Swipe = ( Gesture_Continuous_None + 1),

	Gesture_Continuous_COUNT =  ( Gesture_Continuous_Swipe + 1)
} NuiContinuousGestureType;

struct NuiContinuousGestureResult
{
	float fProgress;

	NuiContinuousGestureResult()
		: fProgress(0.0f)
	{};
};

class NuiGestureResult
{
public:
	NuiGestureResult();
	~NuiGestureResult(){};

	void SetDiscreteGestureResult(NuiDiscreteGestureType type, NuiDiscreteGestureResult result);
	NuiDiscreteGestureResult GetDiscreteGestureResult(NuiDiscreteGestureType type) const;

	void SetContinuousGestureResult(NuiContinuousGestureType type, NuiContinuousGestureResult result);
	NuiContinuousGestureResult GetContinuousGestureResult(NuiContinuousGestureType type) const;

private:
	NuiDiscreteGestureResult	m_discreteGestures[Gesture_Discrete_COUNT];
	NuiContinuousGestureResult	m_continuousGestures[Gesture_Continuous_COUNT];
};