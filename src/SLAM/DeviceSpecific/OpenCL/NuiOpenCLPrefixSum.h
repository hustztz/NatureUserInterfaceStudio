#pragma once

#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiOpenCLPrefixSum
{
public:

	NuiOpenCLPrefixSum(unsigned int numElements);
	~NuiOpenCLPrefixSum();

	unsigned int prefixSum(cl_mem d_input, cl_mem d_output);
	unsigned int prefixSum(cl_kernel scan1Kernel, cl_kernel scan2Kernel, cl_mem d_input, cl_mem d_output);
protected:
	void		scanExclusiveLocal1(cl_mem d_input, cl_kernel scanKernel= NULL);
	void		scanExclusiveLocal2(unsigned int batchSize, cl_mem d_input, cl_kernel scanKernel = NULL);
	void		uniformUpdate(unsigned int batchSize);
	void		generateValidIDs(cl_mem d_output);
	unsigned int getValidCount();
private:
	unsigned int	m_numElements;
	cl_mem			m_buffer;
	cl_mem			m_outPrefixCL;
};