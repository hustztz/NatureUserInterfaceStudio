#pragma once

#include "OpenCL/NuiKinfuOpenCLFrame.h"
#include "OpenCL/NuiKinfuOpenCLDepthTracker.h"
#include "OpenCL/NuiKinfuOpenCLCameraState.h"
#include "CPU/NuiKinfuCPUFrame.h"
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
				*pFrame = new NuiKinfuOpenCLFrame();
				(*pFrame)->AcquireBuffers(nWidth, nHeight, nColorWidth, nColorHeight);

				*pTracker = new NuiKinfuOpenCLDepthTracker(trackerConfig, nWidth, nHeight);
				*pCameraState = new NuiKinfuCameraState(new NuiKinfuOpenCLCameraState());
			}
			/*{
				*pFrame = new NuiKinfuCPUFrame();
				(*pFrame)->AcquireBuffers(nWidth, nHeight, nColorWidth, nColorHeight);

				*pTracker = new NuiKinfuCPUDepthTracker(trackerConfig, nWidth, nHeight);
				*pCameraState = new NuiKinfuCameraState(NULL);
			}*/
		}

	private:
		NuiKinfuTrackingFactory() {}
	};
}