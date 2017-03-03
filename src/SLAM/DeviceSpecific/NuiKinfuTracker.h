#pragma once

#include <Eigen/Geometry>

class NuiKinfuFrame;
class NuiKinfuFeedbackFrame;
class NuiKinfuCameraState;

class NuiKinfuTracker
{
public:
	virtual bool	EstimatePose(
		NuiKinfuFrame* pFrame,
		NuiKinfuFeedbackFrame* pFeedbackFrame,
		NuiKinfuCameraState* pCameraState,
		Eigen::Affine3f *hint
		) = 0;

	virtual bool	hasColorData() const { return false; }
	virtual bool	log(const std::string& fileName) const { return false; }
	virtual float	getError() const { return 0.0; }
	virtual int		getCount() const { return 0; }
};