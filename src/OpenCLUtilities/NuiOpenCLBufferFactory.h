#pragma once

#include <clew/clew.h>
#include "NuiMappable.h"
#include "NuiTextureMappable.h"

typedef cl_mem (*FN_asUInt32IndexBufferCL)(NuiMappableui&);
typedef cl_mem (*FN_asPosition3fBufferCL)(NuiMappable3f&);
typedef cl_mem (*FN_asColor4fBufferCL)(NuiMappable4f&);
typedef cl_mem (*FN_asNormal3fBufferCL)(NuiMappable3f&);
typedef cl_mem (*FN_asPatchUV2fBufferCL)(NuiMappable2f&);
typedef cl_mem (*FN_asTexture1fBufferCL)(NuiMappablef&);
typedef cl_mem (*FN_asTexture2DCL)(NuiTextureMappable&);
typedef cl_mem (*FN_asFrameTexture2DCL)(NuiTextureMappable&);
typedef cl_mem (*FN_asRenderBufferCL)(NuiTextureMappable&);

class NuiOpenCLBufferFactory
{
public:
    static cl_mem asUInt32IndexBufferCL(NuiMappableui& m);
    static cl_mem asPosition3fBufferCL(NuiMappable3f& m);
	static cl_mem asColor4fBufferCL(NuiMappable4f& m);
	static cl_mem asNormal3fBufferCL(NuiMappable3f& m);
    static cl_mem asPatch2fBufferCL(NuiMappable2f& m);
    static cl_mem asTexture1fBufferCL(NuiMappablef& m);
	static cl_mem asTexture2DCL(NuiTextureMappable& m);
	static cl_mem asFrameTexture2DCL(NuiTextureMappable& m);
	static cl_mem asRenderBufferCL(NuiTextureMappable& m);

public:
    static void RegisterAsUInt32IndexBufferCLFn(FN_asUInt32IndexBufferCL f)
    {
        pAsUInt32IndexBufferCLFn = f;
    }
    static void RegisterAsPosition3fBufferCLFn(FN_asPosition3fBufferCL f)
    {
        pAsPosition3fBufferCLFn = f;
    }
	static void RegisterAsColor4fBufferCLFn(FN_asColor4fBufferCL f)
	{
		pAsColor4fBufferCLFn = f;
	}
	static void RegisterAsNormal3fBufferCLFn(FN_asNormal3fBufferCL f)
	{
		pAsNormal3fBufferCLFn = f;
	}
    static void RegisterAsPatchUV2fBufferCLFn(FN_asPatchUV2fBufferCL f)
    {
        pAsPatchUV2fBufferCLFn = f;
    }
    static void RegisterAsTexture1fBufferCLFn(FN_asTexture1fBufferCL f)
    {
        pAsTexture1fBufferCLFn = f;
    }

	static void RegisterAsTexture2DCLFn(FN_asTexture2DCL f)
	{
		pAsTexture2DCLFn = f;
	}
	static void RegisterAsFrameTexture2DCLFn(FN_asFrameTexture2DCL f)
	{
		pAsFrameTexture2DCLFn = f;
	}
	static void RegisterAsRenderBufferCLFn(FN_asRenderBufferCL f)
	{
		pAsRenderBufferCLFn = f;
	}

private:
    static FN_asUInt32IndexBufferCL pAsUInt32IndexBufferCLFn;
    static FN_asPosition3fBufferCL pAsPosition3fBufferCLFn;
	static FN_asColor4fBufferCL pAsColor4fBufferCLFn;
	static FN_asNormal3fBufferCL pAsNormal3fBufferCLFn;
    static FN_asPatchUV2fBufferCL pAsPatchUV2fBufferCLFn;
    static FN_asTexture1fBufferCL pAsTexture1fBufferCLFn;
	static FN_asTexture2DCL pAsTexture2DCLFn;
	static FN_asFrameTexture2DCL pAsFrameTexture2DCLFn;
	static FN_asRenderBufferCL pAsRenderBufferCLFn;
};
