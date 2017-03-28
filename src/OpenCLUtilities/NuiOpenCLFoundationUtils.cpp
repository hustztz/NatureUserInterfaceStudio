#include "NuiOpenCLFoundationUtils.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "Foundation/NuiDebugMacro.h"

namespace NuiOpenCLFoundationUtils
{
	void resetUcharBuffer(cl_mem buffer, UINT num)
	{
		// Get the kernel
		cl_kernel resetKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_RESET_UCHAR_BUFFER);
		if (!resetKernel)
		{
			NUI_ERROR("Get kernel 'E_RESET_UCHAR_BUFFER' failed!\n");
			return;
		}

		// OpenCL command queue and device
		cl_int           err = CL_SUCCESS;
		cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(resetKernel, idx++, sizeof(cl_mem), &buffer);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate
		size_t kernelGlobalSize[1] = { num };
		err = clEnqueueNDRangeKernel(
			queue,
			resetKernel,
			1,
			nullptr,
			kernelGlobalSize,
			nullptr,
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);
	}

	void resetShort3Buffer(cl_mem buffer, UINT num)
	{
		// Get the kernel
		cl_kernel resetKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_RESET_SHORT3_BUFFER);
		if (!resetKernel)
		{
			NUI_ERROR("Get kernel 'E_RESET_SHORT3_BUFFER' failed!\n");
			return;
		}

		// OpenCL command queue and device
		cl_int           err = CL_SUCCESS;
		cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(resetKernel, idx++, sizeof(cl_mem), &buffer);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate
		size_t kernelGlobalSize[1] = { num };
		err = clEnqueueNDRangeKernel(
			queue,
			resetKernel,
			1,
			nullptr,
			kernelGlobalSize,
			nullptr,
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);
	}

	void setFloat2Buffer(float first, float second, cl_mem buffer, UINT num)
	{
		// Get the kernel
		cl_kernel setKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_SET_FLOAT2_BUFFER);
		if (!setKernel)
		{
			NUI_ERROR("Get kernel 'E_SET_FLOAT2_BUFFER' failed!\n");
			return;
		}

		// OpenCL command queue and device
		cl_int           err = CL_SUCCESS;
		cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

		// Set kernel arguments
		cl_uint idx = 0;
		err = clSetKernelArg(setKernel, idx++, sizeof(cl_mem), &buffer);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(setKernel, idx++, sizeof(cl_float), &first);
		NUI_CHECK_CL_ERR(err);
		err = clSetKernelArg(setKernel, idx++, sizeof(cl_float), &second);
		NUI_CHECK_CL_ERR(err);

		// Run kernel to calculate
		size_t kernelGlobalSize[1] = { num };
		err = clEnqueueNDRangeKernel(
			queue,
			setKernel,
			1,
			nullptr,
			kernelGlobalSize,
			nullptr,
			0,
			NULL,
			NULL
			);
		NUI_CHECK_CL_ERR(err);
	}
}