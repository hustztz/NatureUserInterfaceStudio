#include "NuiPrefixSum.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

//All three kernels run 512 threads per workgroup
//Must be a power of two
#define   WORKGROUP_SIZE  256

NuiPrefixSum::NuiPrefixSum()
{
	
}

NuiPrefixSum::~NuiPrefixSum()
{
	
}

unsigned int NuiPrefixSum::prefixSum(unsigned int numElements, cl_mem d_input, cl_mem d_output)
{
	// Get the kernel
	cl_kernel scanKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_PREFIX_SUM);
	//assert(scanKernel);
	if (!scanKernel)
	{
		NUI_ERROR("Get kernel 'E_PREFIX_SUM' failed!\n");
		return 0;
	}

	numElements = (numElements + (4 * WORKGROUP_SIZE - 1)) / (4 * WORKGROUP_SIZE);

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_mem), &d_input);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_mem), &d_output);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scanKernel, idx++, 2 * WORKGROUP_SIZE * sizeof(cl_uint), NULL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_uint), &numElements);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[1] = { numElements / 4 };
	size_t local_ws[1] = { WORKGROUP_SIZE };
	err = clEnqueueNDRangeKernel(
		queue,
		scanKernel,
		1,
		nullptr,
		kernelGlobalSize,
		local_ws,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	int sum = 0;
	err = clEnqueueReadBuffer(
		queue,
		d_output,
		CL_TRUE,//blocking
		(numElements-1) * sizeof(cl_int), //offset
		sizeof(cl_int),
		&sum,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	return (unsigned int)sum;
}