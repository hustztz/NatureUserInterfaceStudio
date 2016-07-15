#pragma once

#include "OpenCLUtilities/NuiOpenCLUtil.h"

class NuiPrefixSum
{
public:

	NuiPrefixSum();
	~NuiPrefixSum();

	unsigned int prefixSum(unsigned int numElements, cl_mem d_input, cl_mem d_output);

	unsigned int getMaxScanSize();

private:
};