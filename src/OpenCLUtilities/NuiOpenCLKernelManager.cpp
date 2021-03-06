// =======================================================================
// Copyright 2015 Autodesk, Inc. All rights reserved.
//
// This computer source code and related instructions and comments are the
// unpublished confidential  and proprietary information of Autodesk, Inc.
// and are protected under applicable copyright and trade secret law. They 
// may not be disclosed to, copied  or used by any third party without the 
// prior written consent of Autodesk, Inc.
// =======================================================================

#include "NuiOpenCLKernelManager.h"
#include "NuiOpenCLGlobal.h"
#include "Foundation/NuiDebugMacro.h"
#include "Foundation/NuiProfilingScope.h"

#include <algorithm>
#include <cassert>
#include <sstream>

NuiOpenCLKernelManager& NuiOpenCLKernelManager::instance() {
    static NuiOpenCLKernelManager s; return s;
}

static const char * const KernelNames[] =
{
	"reset_uchar_kernel",						  "foundation.cl",
	"reset_short3_kernel",						  "foundation.cl",
	"set_float2_kernel",						  "foundation.cl",
	"invalid_float3_kernel",					  "foundation.cl",
	"scanExclusiveLocal1_kernel",				  "prefixSum.cl",
	"scanExclusiveLocal2_kernel",				  "prefixSum.cl",
	"exclusiveScanFlag1_kernel",				  "prefixSum.cl",
	"exclusiveScanFlag2_kernel",				  "prefixSum.cl",
	"uniformUpdate_kernel",						  "prefixSum.cl",
	"compactifyValidPrefixSum_kernel",			  "prefixSum.cl",
    "estimate_normals_kernel",                    "NormalEstimation.cl",
	"estimate_normals_simple_kernel",             "NormalEstimation.cl",
	"estimate_normals_covariance_kernel",         "NormalEstimation.cl",
	"estimate_normals_covariance_pyr_down_kernel",  "NormalEstimation.cl",
	"smooth_normals_kernel",                      "NormalEstimation.cl",
	"depth_passing_filter_kernel",                 "filters.cl",
	"bilateral_filter_kernel",                    "bilateralFilter.cl",
	"generate_gaussian_kernel",                    "bilateralFilter.cl",
	"bilateral_filter_depth_kernel",               "bilateralFilter.cl",
	"half_sample_kernel",						  "transformMaps.cl",
	"half_sample_bgra_kernel",					  "transformMaps.cl",
	"depth2vertex_kernel",						  "transformMaps.cl",
	"UV2color",									  "transformMaps.cl",
	"transform_maps_kernel",					  "transformMaps.cl",
	"resize_maps_kernel",						  "transformMaps.cl",
	"color_to_texture_kernel",					  "transformMaps.cl",
	"float3_to_texture_kernel",					  "transformMaps.cl",
	"float2_to_texture_kernel",					  "transformMaps.cl",
	"intensity_to_float4_kernel",				  "transformMaps.cl",
	"bgra_to_intensity_kernel",					  "transformMaps.cl",
	"bgra_to_color_kernel",						  "transformMaps.cl",
	"intensity_derivatives_kernel",				  "transformMaps.cl",
	"bgra_gradient_kernel",						  "transformMaps.cl",
	"icp_kernel",								  "icp.cl",
	"icp_block_kernel",							  "icp.cl",
	"intensity_icp_block_kernel",				  "icp.cl",
	"compute_sums",								  "icp.cl",
	"color_difference_kernel",					  "colorIcp.cl",
	"color_icp_block_kernel",					  "colorIcp.cl",
	"compute_color_sums",						  "colorIcp.cl",
	"initializeVolume",							  "tsdfVolume.cl",
	"initializeColorVolume",					  "tsdfVolume.cl",
	"fetchVolumeKernel",						  "extract.cl",
	"fetchShiftVolumeKernel",						  "extract.cl",
	"scaleDepthsKernel",						  "tsdfVolume.cl",
	"integrateTsdfVolumeKernel",				  "tsdfVolume.cl",
	"raycastKernel",							  "raycast.cl",
	"volume2vmapKernel",						  "marchingCube.cl",
	"volumeTraversalKernel",					  "marchingCube.cl",
	"volume2VertexKernel",						  "marchingCube.cl",
	"marchingCubeKernel",						  "marchingCube.cl",
	"reset_voxel_block_kernel",					  "KinfuHashScene.cl",
	"reset_hash_entry_kernel",					  "KinfuHashScene.cl",
	"reset_hash_excess_allocation_kernel",		  "KinfuHashScene.cl",
	"reset_visible_entrys_kernel",				  "KinfuHashScene.cl",
	"buildHashAllocAndVisibleType_kernel",		  "KinfuHashScene.cl",
	"allocateVoxelBlocksList_kernel",			  "KinfuHashScene.cl",
	"buildVisibleList_kernel",					  "KinfuHashScene.cl",
	"buildVisibleEnlargedList_kernel",			  "KinfuHashScene.cl",
	"integrateIntoScene_kernel",				  "KinfuHashScene.cl",
	"fetchHashScene_kernel",					  "KinfuHashVisualization.cl",
	"meshHashScene_kernel",						  "KinfuHashVisualization.cl",
	"project_minmax_depths_kernel",				  "KinfuHashRaycast.cl",
	"raycast_kernel",							  "KinfuHashRaycast.cl",
	"forwardProject_kernel",					  "KinfuHashRaycast.cl",
	"findMissingPoints_kernel",					  "KinfuHashRaycast.cl",
	"genericRaycastMissingPoints_kernel",		  "KinfuHashRaycast.cl",
	"reAllocateSwappedOutVoxelBlocks_kernel",	  "KinfuHashSwapping.cl",
	"prefix_swapin_scan1_kernel",				  "KinfuHashSwapping.cl",
	"prefix_swapin_scan2_kernel",				  "KinfuHashSwapping.cl",
	"integrateOldIntoActiveData_kernel",		  "KinfuHashSwapping.cl",
	"buildSwapOutList_kernel",					  "KinfuHashSwapping.cl",
	"moveActiveDataToTransferBuffer_kernel",	  "KinfuHashSwapping.cl",
	"cleanVoxelMemory_kernel",					  "KinfuHashSwapping.cl",
	"reset_heap_kernel",						  "hashingReset.cl",
	"reset_hash_kernel",						  "hashingReset.cl",
	"reset_hash_bucket_mutex_kernel",			  "hashingReset.cl",
	"alloc_SDFs_kernel",						  "hashingSDFData.cl",
	"fillDecisionArrayKernel",					  "hashingSDFData.cl",
	"compactifyHashKernel",						  "hashingSDFData.cl",
	"integrateDepthMapKernel",					  "hashingSDFData.cl",
	"starveVoxelsKernel",						  "hashingSDFData.cl",
	"garbageCollectIdentifyKernel",				  "hashingSDFData.cl",
	"garbageCollectFreeKernel",					  "hashingSDFData.cl",
	"renderKernel",								  "hashingRaycastSDF.cl",
	"rayIntervalSplatKernel",					  "hashingRaycastSDF.cl",
	"integrateFromGlobalHashPass1Kernel",		  "hashingChunkGrid.cl",
	"integrateFromGlobalHashPass2Kernel",		  "hashingChunkGrid.cl",
	"chunkToGlobalHashPass1Kernel",				  "hashingChunkGrid.cl",
	"chunkToGlobalHashPass2Kernel",				  "hashingChunkGrid.cl",
	"fetch_SDFs_kernel",						  "hashingSDFFetch.cl",
};

NuiOpenCLKernelManager::NuiOpenCLKernelManager()
{
    // Initialize kernel vector
    _kernels.resize(E_KERNEL_COUNT);
    for (auto i = 0; i < E_KERNEL_COUNT; ++i){
        _kernels[i] = nullptr;
    }
}


cl_kernel NuiOpenCLKernelManager::acquireKernel(EXGenOCLKernelName name)
{
    // Validate kernel name
    assert(name < E_KERNEL_COUNT);
    if (name >= E_KERNEL_COUNT){
        NUI_ERROR("NuiOpenCLKernelManager::acquireKernel(%d) - invalid kernel name\n", (int)name);
        return nullptr;
    }

    // Just return the kernel if it exists
    if (_kernels[name] && _kernels[name]->kernel) {
        return _kernels[name]->kernel;
    }

    // Error messages
    std::stringstream ss;

    // Create a new kernel and then add it to map
    std::shared_ptr<openclutil::OpenCLKernelInfo> clKnlInfo;

    const char* const kernelname = KernelNames[name * 2];
    const char* const filename = KernelNames[name * 2 + 1];
    auto pIt = _programs.find(filename);
    if ((pIt == _programs.end())||(!pIt->second)) {
        clKnlInfo = openclutil::buildCLKernelFromFile(kernelname, filename, ss);
    }
    else {
        clKnlInfo = openclutil::buildCLKernelFromProgram(kernelname, pIt->second, ss);
    }

    if (clKnlInfo) {
        _kernels[name] = clKnlInfo;
        if (clKnlInfo->program) {
            _programs[filename] = clKnlInfo->program;
        }
        NUI_DEBUG("Kernel '%s' build successfully!\n", kernelname);
    }
    else {
        // Write log to file if there is any
        std::string buildLog = ss.str();
        if (buildLog.size() != 0) {
            NuiOpenCLGlobal::instance().dumpFileToTmpDir(
                std::string(filename) + "___" + kernelname + "___buildLog.log", buildLog);
        }

        NUI_ERROR("Build CL kernel '%s' failed!"
            " Please check build log at '%s'.\n", kernelname,
            NuiOpenCLGlobal::instance().tmpDir().c_str());
    }

    return clKnlInfo?clKnlInfo->kernel:nullptr;
}

void NuiOpenCLKernelManager::buildAllShaders()
{
    NuiProfilingScope profiler("NuiOpenCLKernelManager::buildAllShaders()");

    for (auto i = 0; i < E_KERNEL_COUNT; ++i){
        acquireKernel((EXGenOCLKernelName)i);
    }
}

void NuiOpenCLKernelManager::reset()
{
    // release kernels
    std::fill(_kernels.begin(), _kernels.end(), nullptr);

    // release programs
    for (const auto& it : _programs) {
        if (it.second) {
            cl_int err = clReleaseProgram(it.second);
            NUI_CHECK_CL_ERR(err);
        }
    }
    _programs.clear();

    NUI_DEBUG("All programs/kernels rested.\n");
}

