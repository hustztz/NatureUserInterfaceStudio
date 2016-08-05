#include "NuiOpenCLBufferFactory.h"
#include <assert.h>

FN_asUInt32IndexBufferCL NuiOpenCLBufferFactory::pAsUInt32IndexBufferCLFn = nullptr;
FN_asPosition3fBufferCL NuiOpenCLBufferFactory::pAsPosition3fBufferCLFn = nullptr;
FN_asColor4fBufferCL NuiOpenCLBufferFactory::pAsColor4fBufferCLFn = nullptr;
FN_asNormal3fBufferCL NuiOpenCLBufferFactory::pAsNormal3fBufferCLFn = nullptr;
FN_asPatchUV2fBufferCL NuiOpenCLBufferFactory::pAsPatchUV2fBufferCLFn = nullptr;
FN_asTexture1fBufferCL NuiOpenCLBufferFactory::pAsTexture1fBufferCLFn = nullptr;
FN_asTexture2DCL NuiOpenCLBufferFactory::pAsTexture2DCLFn = nullptr;

cl_mem NuiOpenCLBufferFactory::asUInt32IndexBufferCL(NuiMappableui& m)
{
    assert(pAsUInt32IndexBufferCLFn);
    if (!pAsUInt32IndexBufferCLFn) {
        return nullptr;
    }
    return pAsUInt32IndexBufferCLFn(m);
}

cl_mem NuiOpenCLBufferFactory::asPosition3fBufferCL(NuiMappable3f& m)
{
    assert(pAsPosition3fBufferCLFn);
    if (!pAsPosition3fBufferCLFn) {
        return nullptr;
    }
    return pAsPosition3fBufferCLFn(m);
}

cl_mem NuiOpenCLBufferFactory::asColor4fBufferCL(NuiMappable4f& m)
{
	assert(pAsColor4fBufferCLFn);
	if (!pAsColor4fBufferCLFn) {
		return nullptr;
	}
	return pAsColor4fBufferCLFn(m);
}

cl_mem NuiOpenCLBufferFactory::asNormal3fBufferCL(NuiMappable3f& m)
{
	assert(pAsNormal3fBufferCLFn);
	if (!pAsNormal3fBufferCLFn) {
		return nullptr;
	}
	return pAsNormal3fBufferCLFn(m);
}

cl_mem NuiOpenCLBufferFactory::asPatch2fBufferCL(NuiMappable2f& m)
{
    assert(pAsPatchUV2fBufferCLFn);
    if (!pAsPatchUV2fBufferCLFn) {
        return nullptr;
    }
    return pAsPatchUV2fBufferCLFn(m);
}

cl_mem NuiOpenCLBufferFactory::asTexture1fBufferCL(NuiMappablef& m)
{
    assert(pAsTexture1fBufferCLFn);
    if (!pAsTexture1fBufferCLFn) {
        return nullptr;
    }
    return pAsTexture1fBufferCLFn(m);
}

cl_mem NuiOpenCLBufferFactory::asTexture2DCL(NuiTextureMappable& m)
{
	assert(pAsTexture2DCLFn);
	if (!pAsTexture2DCLFn) {
		return nullptr;
	}
	return pAsTexture2DCLFn(m);
}
