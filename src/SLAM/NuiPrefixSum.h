#pragma once

#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiPrefixSum
{
public:

	NuiPrefixSum();
	~NuiPrefixSum();

	unsigned int prefixSum(unsigned int numElements, cl_mem d_input, cl_mem d_output);

	unsigned int getMaxScanSize();
protected:
	void		scanExclusiveLocal1(unsigned int numElements, cl_mem d_input, cl_mem d_output);
	void		scanExclusiveLocal2(unsigned int numElements, cl_mem d_input, cl_mem d_output);
	void		uniformUpdate(unsigned int numElements, cl_mem d_output);
private:
	cl_mem		m_buffer;
};