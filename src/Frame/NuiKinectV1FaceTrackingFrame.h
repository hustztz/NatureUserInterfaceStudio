#pragma once

#include "NuiImageFrame.h"
#include "NuiCompressedDepthFrame.h"

struct NuiKinectV1FaceTrackingFrame
{
	NuiColorFrame						fColorFrameBuffer;
	NuiCompressedDepthFrame				fCompressedDepthBuffer;
};