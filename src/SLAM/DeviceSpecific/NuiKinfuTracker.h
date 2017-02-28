#pragma once

#include <Eigen/Geometry>

class NuiKinfuFrame;
class NuiKinfuCameraState;
class NuiKinfuScene;
class NuiCLMappableData;

class NuiKinfuTracker
{
public:
	enum BufferType
	{
		eTracking_Vertices = 0,
		eTracking_Normals,
	};

	virtual bool	EvaluateFrame(NuiKinfuFrame* pFrame, NuiKinfuCameraState* pCameraState) = 0;
	virtual bool	EstimatePose(NuiKinfuCameraState* pCameraState, Eigen::Affine3f *hint) = 0;
	virtual void	FeedbackPose(NuiKinfuCameraState* pCameraState) = 0;
	virtual void	FeedbackPose(NuiKinfuCameraState* pCameraState, NuiKinfuScene* pScene) = 0;

	virtual bool	VerticesToMappablePosition(NuiCLMappableData* pMappableData) { return false; }
	virtual bool	BufferToMappableTexture(NuiCLMappableData* pMappableData, BufferType bufferType) { return false; }

	virtual bool	hasColorData() const { return false; }
	virtual bool	log(const std::string& fileName) const { return false; }
	virtual float	getError() const { return 0.0; }
	virtual float	getCount() const { return 0.0; }
};