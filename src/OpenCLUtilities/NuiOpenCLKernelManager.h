#pragma once

#include <clew/clew.h>
#include <vector>
#include <map>
#include <memory>

#include "NuiOpenCLUtil.h"


enum EXGenOCLKernelName{
	E_RESET_UCHAR_BUFFER,
	E_SET_FLOAT2_BUFFER,
	E_PREFIX_SUM_EXCLUSIVE1,
	E_PREFIX_SUM_EXCLUSIVE2,
	E_PREFIX_SUM_UNIFORM_UPDATE,
	E_PREFIX_COMPACTIFY_VALID_SUM,
    E_ESTIMATE_NORMALS,
	E_ESTIMATE_NORMALS_SIMPLE,
	E_ESTIMATE_NORMALS_CONVARIANCE,
	E_ESTIMATE_NORMALS_CONVARIANCE_PYR_DOWN,
	E_SMOOTH_NORMALS,
	E_PASSING_FILTER,
	E_BILATERAL_FILTER,
	E_GENERATE_GAUSSIAN,
	E_BILATERAL_FILTER_DEPTH,
	E_ESTIMATE_HALF_SAMPLE,
	E_ESTIMATE_HALF_BGRA_SAMPLE,
	E_DEPTH2VERTEX,
	E_UV2COLOR,
	E_TRANSFORM_MAPS,
	E_RESIZE_MAPS,
	E_FLOAT3_TO_TEXTURE,
	E_FLOAT2_TO_TEXTURE,
	E_INTENSITY_TO_FLOAT4,
	E_BGRA_TO_INTENSITY,
	E_RGBA_TO_COLOR,
	E_INTENSITY_DERIVATIVES,
	E_BGRA_GRADIENT,
	E_ICP,
	E_ICP_BLOCK,
	E_INTENSITY_ICP_BLOCK,
	E_ICP_SUM,
	E_COLOR_DIFFERENCE,
	E_COLOR_ICP_BLOCK,
	E_COLOR_ICP_SUM,
	E_INITIALIZE_VOLUME,
	E_INITIALIZE_COLOR_VOLUME,
	E_FETCH_VOLUME,
	E_SCALE_DEPTHS,
	E_INTEGRATE_TSDF_VOLUME,
	E_RAY_CAST,
	E_VOLUME2VMAP,
	E_VOLUME_TRAVERSAL,
	E_VOLUME2VERTEX,
	E_MARCHING_CUBE,

	E_HASHING_INITIALIZE_VOXEL_BLOCK,
	E_HASHING_INITIALIZE_HASHENTRY,
	E_HASHING_INITIALIZE_EXCESS_ALLOCATION,
	E_HASHING_RESET_VISIBLE_ENTRYS,
	E_HASHING_BUILD_HASH_ALLOC,
	E_HASHING_ALLOC_VOXEL_BLOCKS,
	E_HASHING_BUILD_VISIBLE_LIST,
	E_HASHING_BUILD_VISIBLE_ENLARGED_LIST,
	E_HASHING_PREFIX_FLAG_SCAN1,
	E_HASHING_PREFIX_FLAG_SCAN2,
	E_HASHING_REALLOC_SWAPPEDOUT_VOXEL,
	E_HASHING_INTEGRATE_INTO_SCENE,
	E_HASHING_FETCH_SCENE,
	E_HASHING_PROJECT_MINMAX_DEPTHS,
	E_HASHING_RAYCAST_RENDER,
	E_HASHING_FORWARD_PROJECT_RENDER,

	E_HASHING_INITIALIZE_HEAP,
	E_HASHING_INITIALIZE_HASH,
	E_HASHING_INITIALIZE_HASH_Bucket_Mutex,
	E_HASHING_ALLOC_SDF,
	E_HASHING_FILL_DECISION_ARRAY,
	E_HASHING_COMPACTIFY_HASH,
	E_HASHING_INTEGRATE_DEPTH_MAP,
	E_HASHING_STARVE_VOXELS,
	E_HASHING_GARBAGE_COLLECT_INDENTIFY,
	E_HASHING_GARBAGE_COLLECT_FREE,
	E_HASHING_RAYCAST_SDF,
	E_HASHING_RAY_INTERVAL_SPLAT,
	E_HASHING_GPU_TO_CPU_PASS1,
	E_HASHING_GPU_TO_CPU_PASS2,
	E_HASHING_CPU_TO_GPU_PASS1,
	E_HASHING_CPU_TO_GPU_PASS2,
	E_HASHING_FETCH_VOLUME,
    E_KERNEL_COUNT
};

// Simple singleton class to manage opencl kernerls
class NuiOpenCLKernelManager
{
public:
    static NuiOpenCLKernelManager& instance();
    cl_kernel acquireKernel(EXGenOCLKernelName name);
    void buildAllShaders();
    void reset();

private:
    std::vector< std::shared_ptr<openclutil::OpenCLKernelInfo> > _kernels;
    std::map<std::string, cl_program> _programs;

private:
    NuiOpenCLKernelManager();
    ~NuiOpenCLKernelManager(){}
    NuiOpenCLKernelManager(const NuiOpenCLKernelManager&){}
    NuiOpenCLKernelManager& operator=(const NuiOpenCLKernelManager&){ return *this; }
};

