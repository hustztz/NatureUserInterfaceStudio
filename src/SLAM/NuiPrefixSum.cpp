#include "NuiPrefixSum.h"

#include "Foundation/NuiDebugMacro.h"
#include "OpenCLUtilities/NuiOpenCLGlobal.h"
#include "OpenCLUtilities/NuiOpenCLKernelManager.h"
#include "OpenCLUtilities/NuiGPUMemManager.h"

#include <assert.h>

//All three kernels run 512 threads per workgroup
//Must be a power of two
#define		WORKGROUP_SIZE		1024
#define		MAX_BATCH_ELEMENTS	8*WORKGROUP_SIZE*WORKGROUP_SIZE

NuiPrefixSum::NuiPrefixSum()
{
	cl_int           err = CL_SUCCESS;
	cl_context       context = NuiOpenCLGlobal::instance().clContext();

	m_buffer = NuiGPUMemManager::instance().CreateBufferCL(context, CL_MEM_READ_WRITE, (MAX_BATCH_ELEMENTS / (8 * WORKGROUP_SIZE)) * sizeof(cl_uint), NULL, &err);
	NUI_CHECK_CL_ERR(err);
}

NuiPrefixSum::~NuiPrefixSum()
{
	if (m_buffer) {
		cl_int err = NuiGPUMemManager::instance().ReleaseMemObjectCL(m_buffer);
		NUI_CHECK_CL_ERR(err);
		m_buffer = NULL;
	}
}

unsigned int iSnapUp(unsigned int dividend, unsigned int divisor){
	return ((dividend % divisor) == 0) ? dividend : (dividend - dividend % divisor + divisor);
}

void NuiPrefixSum::scanExclusiveLocal1(unsigned int numElements, cl_mem d_input, cl_mem d_output)
{
	// Get the kernel
	cl_kernel scanKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_PREFIX_SUM_EXCLUSIVE1);
	assert(scanKernel);
	if (!scanKernel)
	{
		NUI_ERROR("Get kernel 'E_PREFIX_SUM_EXCLUSIVE1' failed!\n");
		return;
	}

	const unsigned int cArrayLength = WORKGROUP_SIZE * 8;
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
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_uint), &cArrayLength);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	size_t kernelGlobalSize[1] = { numElements / 8 };
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
}

void NuiPrefixSum::scanExclusiveLocal2(unsigned int numElements, cl_mem d_input, cl_mem d_output)
{
	// Get the kernel
	cl_kernel scanKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_PREFIX_SUM_EXCLUSIVE2);
	assert(scanKernel);
	if (!scanKernel)
	{
		NUI_ERROR("Get kernel 'E_PREFIX_SUM_EXCLUSIVE2' failed!\n");
		return;
	}
	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_mem), &d_input);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_mem), &m_buffer);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_mem), &d_output);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scanKernel, idx++, 2 * WORKGROUP_SIZE * sizeof(cl_uint), NULL);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_uint), &numElements);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate 
	// iSnapUp
	size_t kernelGlobalSize[1] = { iSnapUp(numElements, WORKGROUP_SIZE) };
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

	// Debug
	//unsigned int* pBuffer = new unsigned int[numElements];
	//err = clEnqueueReadBuffer(
	//	queue,
	//	m_buffer,
	//	CL_TRUE,//blocking
	//	0, //offset
	//	numElements * sizeof(cl_uint),
	//	pBuffer,
	//	0,
	//	NULL,
	//	NULL
	//	);
	//NUI_CHECK_CL_ERR(err);
	//for(unsigned int i = 0; i < numElements; i++)
	//{
	//	unsigned int hustztz = pBuffer[i];
	//	//std::cout << pBuffer[i]  << std::endl;
	//}
	//delete[] pBuffer;
}

void NuiPrefixSum::uniformUpdate(unsigned int numElements, cl_mem d_output)
{
	// Get the kernel
	cl_kernel scanKernel = NuiOpenCLKernelManager::instance().acquireKernel(E_PREFIX_SUM_UNIFORM_UPDATE);
	assert(scanKernel);
	if (!scanKernel)
	{
		NUI_ERROR("Get kernel 'E_PREFIX_SUM_UNIFORM_UPDATE' failed!\n");
		return;
	}

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	// Set kernel arguments
	cl_uint idx = 0;
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_mem), &m_buffer);
	NUI_CHECK_CL_ERR(err);
	err = clSetKernelArg(scanKernel, idx++, sizeof(cl_mem), &d_output);
	NUI_CHECK_CL_ERR(err);

	// Run kernel to calculate
	size_t kernelGlobalSize[1] = { numElements * WORKGROUP_SIZE };
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
}

unsigned int iFactorRadix2Up(unsigned int num)
{
	unsigned int L = num;
	unsigned int log2L = 0;
	for(; L > 1; L >>= 1)
	{
		log2L ++;
	}
	if(L == 0)
	{
		return num;
	}
	L = 1;
	for (unsigned int i = 0; i < log2L; i ++)
	{
		L <<= 1;
	}
	return L;
}

unsigned int NuiPrefixSum::prefixSum(unsigned int numElements, cl_mem d_input, cl_mem d_output)
{
	const unsigned int cArrayLength = WORKGROUP_SIZE * 8;
	numElements = iFactorRadix2Up(numElements / 8) * 8;

	assert(numElements >= cArrayLength && numElements <= MAX_BATCH_ELEMENTS);
	if(numElements < cArrayLength || numElements > MAX_BATCH_ELEMENTS)
		return 0;

	scanExclusiveLocal1(numElements, d_input, d_output);
	unsigned int batchSize = numElements / cArrayLength;
	scanExclusiveLocal2(batchSize, d_input, d_output);
	uniformUpdate(batchSize, d_output);

	// OpenCL command queue and device
	cl_int           err = CL_SUCCESS;
	cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

	unsigned int sum = 0;
	err = clEnqueueReadBuffer(
		queue,
		d_output,
		CL_TRUE,//blocking
		(numElements-1) * sizeof(cl_uint), //offset
		sizeof(cl_uint),
		&sum,
		0,
		NULL,
		NULL
		);
	NUI_CHECK_CL_ERR(err);

	return sum;
}