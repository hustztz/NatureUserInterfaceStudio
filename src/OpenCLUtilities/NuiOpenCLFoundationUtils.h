#include "OpenCLUtilities/NuiOpenCLUtil.h"

namespace NuiOpenCLFoundationUtils
{
	void resetUcharBuffer(cl_mem buffer, UINT num);
	void resetShort3Buffer(cl_mem buffer, UINT num);
	void setFloat2Buffer(float first, float second, cl_mem buffer, UINT num);
}