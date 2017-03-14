#pragma once

#include "OpenCL/NuiKinfuOpenCLFrame.h"
#include "OpenCL/NuiKinfuOpenCLFeedbackFrame.h"
#include "OpenCL/NuiKinfuOpenCLDepthTracker.h"
#include "OpenCL/NuiKinfuOpenCLColorTracker.h"
#include "OpenCL/NuiKinfuOpenCLCameraState.h"
#include "CPU/NuiKinfuCPUFrame.h"
#include "CPU/NuiKinfuCPUFeedbackFrame.h"
#include "CPU/NuiKinfuCPUDepthTracker.h"

namespace NuiKinfuEngine
{
	class NuiKinfuTrackingFactory
	{
	public:
		static NuiKinfuTrackingFactory& Instance()
		{
			static NuiKinfuTrackingFactory s_instance;
			return s_instance;
		}

		static void BuildTrackingEngine(
			NuiKinfuTracker**		pTracker,
			NuiKinfuFrame**			pFrame,
			NuiKinfuFeedbackFrame**	pFeedbackFrame,
			NuiKinfuCameraState**	pCameraState,
			const NuiTrackerConfig& trackerConfig,
			unsigned int nWidth, unsigned int nHeight,
			unsigned int nColorWidth, unsigned int nColorHeight)
		{
			SafeDelete(*pFrame);
			SafeDelete(*pTracker);
			SafeDelete(*pCameraState);

			// OpenCL
			{
				*pFrame = new NuiKinfuOpenCLFrame(trackerConfig, nWidth, nHeight, nColorWidth, nColorHeight);
				*pFeedbackFrame = new NuiKinfuOpenCLFeedbackFrame(nWidth, nHeight);
				*pTracker = new NuiKinfuOpenCLColorTracker(trackerConfig, nWidth, nHeight);
				*pCameraState = new NuiKinfuOpenCLCameraState();
			}
			/*{
				*pFrame = new NuiKinfuCPUFrame(trackerConfig, nWidth, nHeight);
				*pFeedbackFrame = new NuiKinfuCPUFeedbackFrame(nWidth, nHeight);
				*pTracker = new NuiKinfuCPUDepthTracker(trackerConfig, nWidth, nHeight);
				*pCameraState = new NuiKinfuCameraState();
			}*/
		}

	private:
		NuiKinfuTrackingFactory() {}
	};
}