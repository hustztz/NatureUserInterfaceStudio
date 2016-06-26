#pragma once

#include <clew/clew.h>

#include <memory>
#include <string>
#include <vector>
#include <sstream>

#define NUI_CHECK_CL_ERR(s) \
    if(!openclutil::checkCLErrorStatus((s))){ \
        fprintf(stderr, ">%s(%d): error: '%s'\n", __FILE__, __LINE__, __FUNCTION__); \
    }

#define NUI_CHECK_CL_ERR_RET_NULLPTR(s) \
    if(!openclutil::checkCLErrorStatus((s))){ \
        fprintf(stderr, ">%s(%d): error: '%s'\n", __FILE__, __LINE__, __FUNCTION__); \
        return nullptr; \
    }

#define NUI_CHECK_CL_ERR_STREAM(s, ss) \
    openclutil::checkCLErrorStatusWithStream((s), (ss));

#define NUI_CHECK_CL_ERR_STREAM_RET_NULLPTR(s, ss) \
    if(!openclutil::checkCLErrorStatusWithStream((s), (ss))){ \
        return nullptr; \
    }

namespace openclutil
{
    // OpenCL kernel and its program
    struct OpenCLKernelInfo
    {
        cl_program  program;
        cl_kernel   kernel;
        size_t      workGroupSize;

        OpenCLKernelInfo(cl_program p, cl_kernel k, size_t s)
            : program(p), kernel(k), workGroupSize(s)
        {}
    };

    extern bool checkCLErrorStatus(cl_int err);

    extern bool isUsingNVOpenCLDevice();

    // Build a kernel from program
    extern std::shared_ptr<OpenCLKernelInfo> buildCLKernelFromProgram(
        const std::string&  kernelName,
        cl_program prog,
        std::stringstream& errStream
        );

    // Build a kernel from source
    extern std::shared_ptr<OpenCLKernelInfo> buildCLKernelFromSource(
        const std::string&  kernelName,
        const std::string&  programString,
        std::stringstream& errStream
        );

    // Build a kernel from a file
    extern std::shared_ptr<OpenCLKernelInfo> buildCLKernelFromFile(
        const std::string&  kernelName,
        const std::string&  programFileName,
        std::stringstream& errStream
        );

    // Compute the global work group size that can be evenly divided by local work group size
    extern size_t computeGlobalWorkSize(const size_t elementCount, const size_t localWorkSize);

    // Create an OpenCL buffer object from a GL/DX object (shared resource)
    extern cl_mem createCLObjectFromHWBuffer(cl_mem_flags flags, void* bufferResHandle);

    // Create an OpenCL buffer object from a GL/DX 3D texture (shared resource)
    extern cl_mem createCLObjectFromHWTexture3D(cl_mem_flags flags, void* textureResHandle);

    // Lock a GL/DX object for OpenCL computation
    extern void enqueueAcquireHWObjects(cl_uint num_objects, const cl_mem* mem_objects,
        cl_uint num_events_in_wait_list = 0, const cl_event* event_wait_list = nullptr, cl_event* event = nullptr);

    // Unlock a GL/DX object for OpenCL computation
    extern void enqueueReleaseHWObjects(cl_uint num_objects, const cl_mem* mem_objects,
        cl_uint num_events_in_wait_list = 0, const cl_event* event_wait_list = nullptr, cl_event* event = nullptr);

    // Get cl mem buffer size
    extern unsigned int getMemObjectSizeCL(cl_mem b);

    // Release a list of event objects
    extern void releaseEvents(cl_uint num_events, cl_event* event_list);

    // Release a list of event objects
    extern void releaseEvents(std::vector<cl_event>& events);

    // Estimate OCL image size
    extern unsigned int estimateImageCLSize(const cl_image_format *f, size_t w, size_t h, size_t d);

    // Set cl kernel args
    extern bool setKernelArgs(cl_kernel kernel, const std::vector< std::pair<size_t, void*> >& args);

    // Utility class to automatically release OpenCL event object
    struct AutoEvent
    {
        cl_event event;

        AutoEvent()
            : event(nullptr)
        {}

        ~AutoEvent()
        {
            if (event)
            {
                cl_int err = clReleaseEvent(event);
                NUI_CHECK_CL_ERR(err);
            }
        }

        operator cl_event*()
        {
            return &event;
        }

        cl_event& operator*()
        {
            return event;
        }

        cl_event release()
        {
            cl_event ret = nullptr;
            std::swap(ret, event);
            return ret;
        }
    };

} // namespace openclutil
