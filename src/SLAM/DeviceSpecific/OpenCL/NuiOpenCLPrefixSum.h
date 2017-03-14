#pragma once

#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiOpenCLPrefixSum
{
public:

	NuiOpenCLPrefixSum();
	~NuiOpenCLPrefixSum();

	unsigned int prefixSum(unsigned int numElements, cl_mem d_input, cl_mem d_output, bool bSrcFlag = false);
protected:
	void		scanExclusiveLocal1(unsigned int numElements, cl_mem d_input, cl_mem d_output, bool bSrcFlag);
	void		scanExclusiveLocal2(unsigned int numElements, cl_mem d_input, cl_mem d_output, bool bSrcFlag);
	void		uniformUpdate(unsigned int numElements, cl_mem d_output);
private:
	cl_mem		m_buffer;
};