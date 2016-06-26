#include "NuiGestureResult.h"

NuiGestureResult::NuiGestureResult()
{
	for (int i = 0; i < Gesture_Discrete_COUNT; ++i)
	{
		m_discreteGestures[i].bDetected = false;
		m_discreteGestures[i].fConfidence = 0.0f;
	}
	for (int i = 0; i < Gesture_Continuous_COUNT; ++i)
	{
		m_continuousGestures[i].fProgress = 0.0f;
	}
}

void NuiGestureResult::SetDiscreteGestureResult(NuiDiscreteGestureType type, NuiDiscreteGestureResult result)
{
	if(type >= Gesture_Discrete_COUNT)
		return;
	m_discreteGestures[type] = result;
}
NuiDiscreteGestureResult NuiGestureResult::GetDiscreteGestureResult(NuiDiscreteGestureType type) const
{
	if(type >= Gesture_Discrete_COUNT)
		return NuiDiscreteGestureResult();
	return m_discreteGestures[type];
}

void NuiGestureResult::SetContinuousGestureResult(NuiContinuousGestureType type, NuiContinuousGestureResult result)
{
	if(type >= Gesture_Continuous_COUNT)
		return;
	m_continuousGestures[type] = result;
}
NuiContinuousGestureResult NuiGestureResult::GetContinuousGestureResult(NuiContinuousGestureType type) const
{
	if(type >= Gesture_Continuous_COUNT)
		return NuiContinuousGestureResult();
	return m_continuousGestures[type];
}