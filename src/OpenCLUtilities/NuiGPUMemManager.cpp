// =======================================================================
// Copyright 2015 Autodesk, Inc. All rights reserved.
//
// This computer source code and related instructions and comments are the
// unpublished confidential  and proprietary information of Autodesk, Inc.
// and are protected under applicable copyright and trade secret law. They 
// may not be disclosed to, copied  or used by any third party without the 
// prior written consent of Autodesk, Inc.
// =======================================================================

#include <assert.h>

#include "NuiGPUMemManager.h"
#include "NuiOpenCLUtil.h"
#include "Foundation/NuiDebugMacro.h"

#ifdef ENABLE_GPU_MEM_DEBUG_OUTPUT
#undef ENABLE_GPU_MEM_DEBUG_OUTPUT
#endif

FN_informRenderHoldGPU        NuiGPUMemManager::pInformMayaHoldGPUFn        = nullptr;
FN_informRenderReleaseGPU     NuiGPUMemManager::pInformMayaReleaseGPUFn     = nullptr;
FN_lockRenderResourceHandle   NuiGPUMemManager::pLockMayaResourceHandleFn   = nullptr;
FN_unlockRenderResourceHandle NuiGPUMemManager::pUnlockMayaResourceHandleFn = nullptr;


NuiGPUMemManager& NuiGPUMemManager::instance()
{
    static NuiGPUMemManager s;
    return s;
}

NuiGPUMemManager::NuiGPUMemManager()
{
}

NuiGPUMemManager::~NuiGPUMemManager()
{
}

bool NuiGPUMemManager::informVMemAllocToRender(unsigned int size, unsigned int* evictedSize)
{
    assert(pInformMayaHoldGPUFn && pInformMayaReleaseGPUFn);
    if (!pInformMayaHoldGPUFn || !pInformMayaReleaseGPUFn) {
        return false;
    }

    return pInformMayaHoldGPUFn(size, evictedSize);
}

bool NuiGPUMemManager::informVMemFreeToRender(unsigned int size)
{
    assert(pInformMayaHoldGPUFn && pInformMayaReleaseGPUFn);
    if (!pInformMayaHoldGPUFn || !pInformMayaReleaseGPUFn) {
        return false;
    }

    return pInformMayaReleaseGPUFn(size);
}

bool NuiGPUMemManager::isValidCLBuffer(cl_mem m)
{
    if (!m)
        return false;

    if (_pureCLBufs.find(m) != _pureCLBufs.end()) {
        return true;
    }

    if (_sharedCLBufs.find(m) != _sharedCLBufs.end()) {
        return true;
    }

    return false;
}

void NuiGPUMemManager::commonPostCreate(cl_mem b, const cl_int* err, const char* n, bool informedMaya)
{
    // Do nothing on creating failure
    if (!b)
        return;

    if (err && (*err != CL_SUCCESS)) {
        NUI_CHECK_CL_ERR(*err);
        return;
    }

    // Record this buffer with size
    unsigned int size = openclutil::getMemObjectSizeCL(b);
    assert(size > 0);

    _pureCLBufs.insert(std::make_pair(b, NuiGPUMem(b, size, n, informedMaya)));

#ifdef ENABLE_GPU_MEM_DEBUG_OUTPUT
        // Debug
        NUI_DEBUG("+ pure   ocl buffer(size= %d byte, name=\"%s\"), current count(pure - %d, shared - %d)\n",
            size, n ? n : "", (int)_pureCLBufs.size(), (int)_sharedCLBufs.size());
#endif
}

cl_mem NuiGPUMemManager::CreateBufferCL(
    cl_context context,
    cl_mem_flags flags,
    size_t size,
    void *host_ptr,
    cl_int *errcode_ret,
    const char* debugName/* = nullptr*/)
{
    // Make sure we have enough GPU memory space
    bool informedMaya = false;
    {
        unsigned int evictedSize = 0;
        informedMaya = informVMemAllocToRender((unsigned int)size, &evictedSize);
        if (!informedMaya){
            NUI_ERROR("Not enough GPU memory for 'clCreateBuffer(size=%d)' !\n", (int)size);
        }
        if (evictedSize > 0) {
            NUI_DEBUG("Maya paged out %d byte of GPU data to CPU.\n", (int)evictedSize);
        }
    }

    // Call into driver
    cl_mem b = clCreateBuffer(context, flags, size, host_ptr, errcode_ret);

    // Common post creating
    commonPostCreate(b, errcode_ret, debugName, informedMaya);

    return b;
}

cl_mem NuiGPUMemManager::CreateSubBufferCL(
	cl_mem buffer,
	cl_mem_flags flags,
	size_t origin,
	size_t size,
	cl_int *errcode_ret,
	const char* debugName/* = nullptr*/)
{
	// Make sure we have enough GPU memory space
	bool informedMaya = false;
	{
		unsigned int evictedSize = 0;
		informedMaya = informVMemAllocToRender((unsigned int)size, &evictedSize);
		if (!informedMaya){
			NUI_ERROR("Not enough GPU memory for 'clCreateBuffer(size=%d)' !\n", (int)size);
		}
		if (evictedSize > 0) {
			NUI_DEBUG("Maya paged out %d byte of GPU data to CPU.\n", (int)evictedSize);
		}
	}

	cl_buffer_region region;
	region.size = size;
	region.origin = origin;
	// Call into driver
	cl_mem b = clCreateSubBuffer(buffer, flags, CL_BUFFER_CREATE_TYPE_REGION, &region, errcode_ret);

	// Common post creating
	commonPostCreate(b, errcode_ret, debugName, informedMaya);

	return b;
}

cl_mem NuiGPUMemManager::CreateImage2DCL(
    cl_context context,
    cl_mem_flags flags,
    const cl_image_format *image_format,
    size_t image_width,
    size_t image_height,
    size_t image_row_pitch,
    void *host_ptr,
    cl_int *errcode_ret,
    const char* debugName/* = nullptr*/)
{
    // Make sure we have enough GPU memory space
    bool informedMaya = false;
    {
        const unsigned int expectedSize = openclutil::estimateImageCLSize(
            image_format, image_width, image_height, 1);
        unsigned int evictedSize = 0;
        informedMaya = informVMemAllocToRender(expectedSize, &evictedSize);

        if (!informedMaya) {
            NUI_ERROR("Not enough GPU memory for 'clCreateImage2D(size=%d)' !\n", (int)expectedSize);
        }
        if (evictedSize > 0) {
            NUI_DEBUG("Maya paged out %d byte of GPU data to CPU.\n", (int)evictedSize);
        }
    }

    // Call into driver
    cl_mem b = clCreateImage2D(
        context, flags, image_format, image_width, image_height,
        image_row_pitch, host_ptr, errcode_ret);

    // Common post creating
    commonPostCreate(b, errcode_ret, debugName, informedMaya);

    return b;
}

cl_mem NuiGPUMemManager::CreateImage3DCL(
    cl_context context,
    cl_mem_flags flags,
    const cl_image_format *image_format,
    size_t image_width,
    size_t image_height,
    size_t image_depth,
    size_t image_row_pitch,
    size_t image_slice_pitch,
    void *host_ptr,
    cl_int *errcode_ret,
    const char* debugName/* = nullptr*/)
{
    // Make sure we have enough GPU memory space
    bool informedMaya = false;
    {
        const unsigned int expectedSize = openclutil::estimateImageCLSize(
            image_format, image_width, image_height, image_depth);
        unsigned int evictedSize = 0;
        informedMaya = informVMemAllocToRender(expectedSize, &evictedSize);
        if (!informedMaya) {
            NUI_ERROR("Not enough GPU memory for 'clCreateImage3D(size=%d)' !\n", (int)expectedSize);
        }
        if (evictedSize > 0) {
            NUI_DEBUG("Maya paged out %d byte of GPU data to CPU.\n", (int)evictedSize);
        }
    }

    // Call into driver
    cl_mem b = clCreateImage3D(
        context, flags, image_format, image_width, image_height, image_depth,
        image_row_pitch, image_slice_pitch, host_ptr, errcode_ret);

    // Common post creating
    commonPostCreate(b, errcode_ret, debugName, informedMaya);

    return b;
}

cl_mem NuiGPUMemManager::CreateCLObjectFromHWBuffer(
    cl_mem_flags flags,
    void* bufobj,
    void* ogsbuf,
    NuiGPUMemSharedType type,
    const char* debugName/* = nullptr*/)
{
    // Sanity check
    assert(bufobj);
    if (!bufobj)
        return nullptr;

    /*assert(ogsbuf);
    if (!ogsbuf)
        return nullptr;*/

    // Lock ogs resource first
    pLockMayaResourceHandleFn(ogsbuf, type);

    // Call into driver
    cl_mem b = openclutil::createCLObjectFromHWBuffer(flags, bufobj);

    if (b) {
        // Record this buffer with size
        unsigned int size = openclutil::getMemObjectSizeCL(b);

        // nv gpu always returns 0 for getting size from graphic shared cl buffers.
        //assert(size > 0);

        _sharedCLBufs.insert(std::make_pair(b, NuiGPUMemShared(b, size, ogsbuf, type, debugName)));
#ifdef ENABLE_GPU_MEM_DEBUG_OUTPUT
        // Debug
        NUI_DEBUG("+ shared ocl buffer(size= %d byte, name=\"%s\"), current count(pure - %d, shared - %d)\n",
            size, debugName? debugName:"", (int)_pureCLBufs.size(), (int)_sharedCLBufs.size());
#endif
    }
    else {
        pUnlockMayaResourceHandleFn(ogsbuf, type);
    }

    return b;
}



cl_int NuiGPUMemManager::ReleaseMemObjectCL(cl_mem& b)
{
    cl_int err = CL_SUCCESS;

    // Find it in the previously created record
    auto it = _pureCLBufs.find(b);
    if (it != _pureCLBufs.end()) {
        // Call into driver
        err = clReleaseMemObject(b);
        NUI_CHECK_CL_ERR(err);

        // Inform Maya gpu mem released
        const unsigned int size = it->second.size();
        const std::string  name = it->second.name();
        if (it->second.informedMaya()) {
            informVMemFreeToRender(size);
        }

        // Remove from map
        _pureCLBufs.erase(it);

#ifdef ENABLE_GPU_MEM_DEBUG_OUTPUT
        // Debug
        NUI_DEBUG("- pure   ocl buffer(size= %d byte, name=\"%s\"), current count(pure - %d, shared - %d)\n",
            size, name.c_str(), (int)_pureCLBufs.size(), (int)_sharedCLBufs.size());
#endif
    }
    else {
        auto it = _sharedCLBufs.find(b);
        if (it == _sharedCLBufs.end()) {
            NUI_ERROR("Trying to release an opencl object which is not tracked by XgGPUMemManager!\n");
        }

        // Call into driver
        err = clReleaseMemObject(b);
        NUI_CHECK_CL_ERR(err);

        if (it != _sharedCLBufs.end()) {
            // unlock ogs resouce handle
            pUnlockMayaResourceHandleFn(it->second.ogsBuf(), it->second.sharedType());

            // size
            const unsigned int size = it->second.size();
            const std::string  name = it->second.name();

            // Remove from map
            _sharedCLBufs.erase(it);

#ifdef ENABLE_GPU_MEM_DEBUG_OUTPUT
            // Debug
            NUI_DEBUG("- shared ocl buffer(size= %d byte, name=\"%s\"), current count(pure - %d, shared - %d)\n",
                size, name.c_str(), (int)_pureCLBufs.size(), (int)_sharedCLBufs.size());
#endif
        }
    }

    b = nullptr;
    return err;
}


