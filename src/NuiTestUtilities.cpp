#include "NuiTestUtilities.h"

#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include <iostream>

#include "SLAM/NuiPrefixSum.h"

namespace NuiTestUtilities
{
	bool	testPrefixSum()
	{
		cl_int           err = CL_SUCCESS;
		cl_context       context = NuiOpenCLGlobal::instance().clContext();
		cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

		static const UINT cInputSize = 512*16*10;
		UINT input[cInputSize];
		for(UINT i = 0; i < cInputSize; i++)
			input[i] = 1; //rand();
		/*std::cout << " input:"  << std::endl;
		for(UINT i = 0; i < sInputSize; i++)
			std::cout << input[i]  << std::endl;*/

		UINT outputCPU[cInputSize];
		for(UINT j = 0; j < cInputSize; j++)
			outputCPU[j] = 0;
		for(UINT j = 1; j < cInputSize; j++)
			outputCPU[j] = input[j - 1] + outputCPU[j - 1];
		/*std::cout << " outputCPU:"  << std::endl;
		for(UINT i = 0; i < sInputSize; i++)
			std::cout << outputCPU[i]  << std::endl;*/

		cl_mem inputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, sizeof(cl_uint)*cInputSize, input, &err);
		NUI_CHECK_CL_ERR(err);
		cl_mem outputCL = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, sizeof(cl_uint)*cInputSize, NULL, &err);
		NUI_CHECK_CL_ERR(err);

		NuiPrefixSum scan;
		scan.prefixSum(cInputSize, inputCL, outputCL);

		UINT outputGPU[cInputSize];
		err = clEnqueueReadBuffer(
			queue,
			outputCL,
			CL_TRUE,//blocking
			0, //offset
			cInputSize * sizeof(cl_int),
			outputGPU,
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);

		std::cout << " outputGPU:"  << std::endl;
		for(UINT i = 0; i < cInputSize; i++)
		{
			//std::cout << outputGPU[i]  << std::endl;
			if(outputCPU[i] != outputGPU[i])
			{
				std::cout << "error."  << std::endl;
				return false;
			}
		}
		return true;
	}
}