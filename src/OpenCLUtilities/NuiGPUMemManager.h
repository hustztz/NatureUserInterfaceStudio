#pragma once

#include <clew/clew.h>
#include <map>
#include <string>

enum class NuiGPUMemSharedType
{
    XG_GPUMEM_SHARED_VB,
    XG_GPUMEM_SHARED_IB,
	XG_GPUMEM_SHARED_Tex
};

// Some Maya helper functions need to be reigistered
typedef bool (*FN_informRenderHoldGPU)(unsigned int, unsigned int*);
typedef bool (*FN_informRenderReleaseGPU)(unsigned int);
typedef void (*FN_lockRenderResourceHandle)(void*, NuiGPUMemSharedType);
typedef void (*FN_unlockRenderResourceHandle)(void*, NuiGPUMemSharedType);

// This class is used to hold a piece of gpu memory information
class NuiGPUMem
{
public:
    NuiGPUMem(cl_mem b, unsigned int s, const char* n, bool informedMaya):
        _buf(b), _size(s), _informedMaya(informedMaya)
    {
        _name = n ? std::string(n) : "";
    }

    cl_mem mem() const { return _buf; }
    unsigned int size() const { return _size; }

    const std::string& name() const{ return _name; }

    bool informedMaya() const { return _informedMaya; }
protected:
    cl_mem _buf;
    unsigned int _size;
    bool _informedMaya;
    std::string _name;
};

// This class is used to hold a piece of gpu mem(shared with graphic) information
class NuiGPUMemShared : public NuiGPUMem
{
public:
    NuiGPUMemShared(cl_mem b, unsigned int s, void* ogs, NuiGPUMemSharedType type, const char* n):
        NuiGPUMem(b, s, n, false), _ogsbuf(ogs), _sharedType(type){}

    void* ogsBuf() const { return _ogsbuf; }
    NuiGPUMemSharedType sharedType() const { return _sharedType; }
protected:
    void* _ogsbuf;
    NuiGPUMemSharedType _sharedType;
};

// This class manages all the gpu memory alloc/free,
// All the opencl buffer/image creating/deleting should go through
// the interfaces of this class.

class NuiGPUMemManager
{
public:
    // Singleton
    static NuiGPUMemManager& instance();

    // Some registered Maya functions
    static void RegisterInformRenderHoldGPU(FN_informRenderHoldGPU fn) { pInformMayaHoldGPUFn = fn; }
    static void RegisterInformRenderReleaseGPU(FN_informRenderReleaseGPU fn) { pInformMayaReleaseGPUFn = fn; }
    static void RegisterLockRenderResourceHandle(FN_lockRenderResourceHandle fn) { pLockMayaResourceHandleFn = fn; }
    static void RegisterUnlockRenderResourceHandle(FN_unlockRenderResourceHandle fn) { pUnlockMayaResourceHandleFn = fn; }

public:
    // Check if it is a valid cl buffer
    bool isValidCLBuffer(cl_mem m);

    // Create cl buffer
    cl_mem CreateBufferCL(
        cl_context context,
        cl_mem_flags flags,
        size_t size,
        void *host_ptr,
        cl_int *errcode_ret,
        const char* debugName = nullptr);

	cl_mem CreateSubBufferCL(
		cl_mem buffer,
		cl_mem_flags flags,
		size_t origin,
		size_t size,
		cl_int *errcode_ret,
		const char* debugName = nullptr);

    // Create cl 2d image
    cl_mem CreateImage2DCL(
        cl_context context,
        cl_mem_flags flags,
        const cl_image_format *image_format,
        size_t image_width,
        size_t image_height,
        size_t image_row_pitch,
        void *host_ptr,
        cl_int *errcode_ret,
        const char* debugName = nullptr);

    // Create cl 3d image
    cl_mem CreateImage3DCL(
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
        const char* debugName = nullptr);

    // Create cl buffer from shared vertex buffer
    cl_mem CreateCLObjectFromHWBuffer(
        cl_mem_flags flags,
        void* bufobj,
        void* ogsbuf,
        NuiGPUMemSharedType type,
        const char* debugName = nullptr
        );

	// Create cl buffer from shared vertex buffer
	cl_mem CreateCLTextureFromHWBuffer(
		cl_mem_flags flags,
		void* bufobj,
		void* ogsbuf,
		NuiGPUMemSharedType type,
		const char* debugName = nullptr
		);

	// Create cl buffer from shared render buffer
	cl_mem CreateCLObjectFromRenderBuffer(
		cl_mem_flags flags,
		void* bufobj,
		void* ogsbuf,
		NuiGPUMemSharedType type,
		const char* debugName = nullptr
		);

    // Release cl mem object
    cl_int ReleaseMemObjectCL(cl_mem& b);

private:
    // Map holding the pure opencl buffers
    std::map<cl_mem, NuiGPUMem>       _pureCLBufs;

    // Map holding the graphic/compute shared buffers
    std::map<cl_mem, NuiGPUMemShared> _sharedCLBufs;

    void commonPostCreate(cl_mem b, const cl_int* err, const char* n, bool informedMaya);

    // Maya helper function to tell Maya gpu memory usage in plugin
    bool informVMemAllocToRender(unsigned int, unsigned int*);
    bool informVMemFreeToRender(unsigned int);

private:
    static FN_informRenderHoldGPU pInformMayaHoldGPUFn;
    static FN_informRenderReleaseGPU pInformMayaReleaseGPUFn;
    static FN_lockRenderResourceHandle pLockMayaResourceHandleFn;
    static FN_unlockRenderResourceHandle pUnlockMayaResourceHandleFn;

private:
    // privates these for singleton mechanism
    NuiGPUMemManager();
    ~NuiGPUMemManager();
    NuiGPUMemManager(const NuiGPUMemManager&) {}
    NuiGPUMemManager& operator=(const NuiGPUMemManager&) { return *this; }
};
