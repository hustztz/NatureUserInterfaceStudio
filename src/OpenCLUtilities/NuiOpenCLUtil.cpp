// =======================================================================
// Copyright 2015 Autodesk, Inc. All rights reserved.
//
// This computer source code and related instructions and comments are the
// unpublished confidential  and proprietary information of Autodesk, Inc.
// and are protected under applicable copyright and trade secret law. They 
// may not be disclosed to, copied  or used by any third party without the 
// prior written consent of Autodesk, Inc.
// =======================================================================

#include "NuiOpenCLUtil.h"

#include <glew/GL/glew.h> // include first before gl.h
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <functional>
#include <sstream>
#include <vector>
#include <assert.h>

#include "Foundation/NuiDebugMacro.h"

#include "NuiOpenGLThread.h"
#include "NuiOpenCLGlobal.h"

static const char* CLErrorCodeStr(cl_int err)
{
    switch (err) {
    case CL_SUCCESS: return "CL_SUCCESS";
    case CL_DEVICE_NOT_FOUND: return "CL_DEVICE_NOT_FOUND";
    case CL_DEVICE_NOT_AVAILABLE: return "CL_DEVICE_NOT_AVAILABLE";
    case CL_COMPILER_NOT_AVAILABLE: return "CL_COMPILER_NOT_AVAILABLE";
    case CL_MEM_OBJECT_ALLOCATION_FAILURE: return "CL_MEM_OBJECT_ALLOCATION_FAILURE";
    case CL_OUT_OF_RESOURCES: return "CL_OUT_OF_RESOURCES";
    case CL_OUT_OF_HOST_MEMORY: return "CL_OUT_OF_HOST_MEMORY";
    case CL_PROFILING_INFO_NOT_AVAILABLE: return "CL_PROFILING_INFO_NOT_AVAILABLE";
    case CL_MEM_COPY_OVERLAP: return "CL_MEM_COPY_OVERLAP";
    case CL_IMAGE_FORMAT_MISMATCH: return "CL_IMAGE_FORMAT_MISMATCH";
    case CL_IMAGE_FORMAT_NOT_SUPPORTED: return "CL_IMAGE_FORMAT_NOT_SUPPORTED";
    case CL_BUILD_PROGRAM_FAILURE: return "CL_BUILD_PROGRAM_FAILURE";
    case CL_MAP_FAILURE: return "CL_MAP_FAILURE";
    case CL_MISALIGNED_SUB_BUFFER_OFFSET: return "CL_MISALIGNED_SUB_BUFFER_OFFSET";
    case CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST: return "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST";

    case CL_INVALID_VALUE: return "CL_INVALID_VALUE";
    case CL_INVALID_DEVICE_TYPE: return "CL_INVALID_DEVICE_TYPE";
    case CL_INVALID_PLATFORM: return "CL_INVALID_PLATFORM";
    case CL_INVALID_DEVICE: return "CL_INVALID_DEVICE";
    case CL_INVALID_CONTEXT: return "CL_INVALID_CONTEXT";
    case CL_INVALID_QUEUE_PROPERTIES: return "CL_INVALID_QUEUE_PROPERTIES";
    case CL_INVALID_COMMAND_QUEUE: return "CL_INVALID_COMMAND_QUEUE";
    case CL_INVALID_HOST_PTR: return "CL_INVALID_HOST_PTR";
    case CL_INVALID_MEM_OBJECT: return "CL_INVALID_MEM_OBJECT";
    case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR: return "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR";
    case CL_INVALID_IMAGE_SIZE: return "CL_INVALID_IMAGE_SIZE";
    case CL_INVALID_SAMPLER: return "CL_INVALID_SAMPLER";
    case CL_INVALID_BINARY: return "CL_INVALID_BINARY";
    case CL_INVALID_BUILD_OPTIONS: return "CL_INVALID_BUILD_OPTIONS";
    case CL_INVALID_PROGRAM: return "CL_INVALID_PROGRAM";
    case CL_INVALID_PROGRAM_EXECUTABLE: return "CL_INVALID_PROGRAM_EXECUTABLE";
    case CL_INVALID_KERNEL_NAME: return "CL_INVALID_KERNEL_NAME";
    case CL_INVALID_KERNEL_DEFINITION: return "CL_INVALID_KERNEL_DEFINITION";
    case CL_INVALID_KERNEL: return "CL_INVALID_KERNEL";
    case CL_INVALID_ARG_INDEX: return "CL_INVALID_ARG_INDEX";
    case CL_INVALID_ARG_VALUE: return "CL_INVALID_ARG_VALUE";
    case CL_INVALID_ARG_SIZE: return "CL_INVALID_ARG_SIZE";
    case CL_INVALID_KERNEL_ARGS: return "CL_INVALID_KERNEL_ARGS";
    case CL_INVALID_WORK_DIMENSION: return "CL_INVALID_WORK_DIMENSION";
    case CL_INVALID_WORK_GROUP_SIZE: return "CL_INVALID_WORK_GROUP_SIZE";
    case CL_INVALID_WORK_ITEM_SIZE: return "CL_INVALID_WORK_ITEM_SIZE";
    case CL_INVALID_GLOBAL_OFFSET: return "CL_INVALID_GLOBAL_OFFSET";
    case CL_INVALID_EVENT_WAIT_LIST: return "CL_INVALID_EVENT_WAIT_LIST";
    case CL_INVALID_EVENT: return "CL_INVALID_EVENT";
    case CL_INVALID_OPERATION: return "CL_INVALID_OPERATION";
    case CL_INVALID_GL_OBJECT: return "CL_INVALID_GL_OBJECT";
    case CL_INVALID_BUFFER_SIZE: return "CL_INVALID_BUFFER_SIZE";
    case CL_INVALID_MIP_LEVEL: return "CL_INVALID_MIP_LEVEL";
    case CL_INVALID_GLOBAL_WORK_SIZE: return "CL_INVALID_GLOBAL_WORK_SIZE";
    default:
        break;
    }

    //unknown error. use thread-unsafe static buffer as fallback
    static char buffer[33];
    sprintf_s(buffer, "%d", err);
    return buffer;
}

namespace
{
    using namespace openclutil;

    void releaseCLKernel(OpenCLKernelInfo* info)
    {
        cl_int err = CL_SUCCESS;
        if (!info) return;

        err = clReleaseKernel(info->kernel);
        NUI_CHECK_CL_ERR(err);

        delete info;
    }

} // anonymous namespace

namespace openclutil
{
    bool checkCLErrorStatus(cl_int err)
    {
        if (err != CL_SUCCESS)
        {
            fprintf(stderr, "XGen: OpenCL error %s\n", CLErrorCodeStr(err));
            return false;
        }
        return true;
    }

    bool checkCLErrorStatusWithStream(cl_int err, std::stringstream& ss)
    {
        if (err != CL_SUCCESS)
        {
            ss << "XGen: OpenCL error: " << CLErrorCodeStr(err) << std::endl;
            return false;
        }
        return true;
    }

    bool isUsingNVOpenCLDevice()
    {
        static bool nvOCLDeviceUsed = true;
        static bool queried = false;
        if (queried){
            return nvOCLDeviceUsed;
        }

        {
            cl_device_id ocl_device_id = NuiOpenCLGlobal::instance().clDeviceId();

            size_t vendorStrLen = 0;
            clGetDeviceInfo(ocl_device_id, CL_DEVICE_VENDOR, 0, nullptr, &vendorStrLen);
            vendorStrLen = vendorStrLen > 0 ? vendorStrLen : 100;
            char* vendor_name = new char[vendorStrLen];

            cl_int result = clGetDeviceInfo(
                ocl_device_id, CL_DEVICE_VENDOR, sizeof(char) * vendorStrLen, (void*)(vendor_name), NULL);

            if (result == CL_SUCCESS){
                if ((vendor_name[0] == 'N') && (vendor_name[1] == 'V')){
                    nvOCLDeviceUsed = true;
                }
                else{
                    nvOCLDeviceUsed = false;
                }
                queried = true;
            }

            delete[] vendor_name;
            vendor_name = nullptr;
        }

        return nvOCLDeviceUsed;
    }

    std::shared_ptr<OpenCLKernelInfo> buildCLKernelFromProgram(
        const std::string&  kernelName,
        cl_program prog,
        std::stringstream& errStream
        )
    {
        cl_int       err = CL_SUCCESS;
        // Create kernel from program
        cl_kernel kernel = clCreateKernel(prog, kernelName.c_str(), &err);
        NUI_CHECK_CL_ERR_STREAM_RET_NULLPTR(err, errStream);

        // Query the local work group size
        size_t workGroupSize = 0;
        size_t retSize = 0;
        cl_device_id device = NuiOpenCLGlobal::instance().clDeviceId();
        err = clGetKernelWorkGroupInfo(kernel, device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(size_t), &workGroupSize, &retSize);
        NUI_CHECK_CL_ERR_STREAM_RET_NULLPTR(err, errStream);

        size_t localSize = (retSize > 0) ? workGroupSize : 256;

        // Done
        return std::shared_ptr<OpenCLKernelInfo>(
            new OpenCLKernelInfo(prog, kernel, localSize),
            std::ptr_fun(releaseCLKernel)
            );
    }

    std::shared_ptr<OpenCLKernelInfo> buildCLKernelFromSource(
        const std::string&  kernelName,
        const std::string&  programString,
        std::stringstream& errStream
        )
    {
        cl_context context = NuiOpenCLGlobal::instance().clContext();
        cl_device_id device = NuiOpenCLGlobal::instance().clDeviceId();

        cl_int       err = CL_SUCCESS;
        cl_program   program = 0;

        // Create the program
        const char* source = programString.c_str();
        size_t      length = programString.length();

        if ((length == 0)||!source) {
            errStream << "Kernel(name=\"" << kernelName << "\")"
                " source content is empty, skip building!" << std::endl;;
            return nullptr;
        }

        program = clCreateProgramWithSource(context, 1, &source, &length, &err);
        NUI_CHECK_CL_ERR_STREAM(err, errStream);

        // Build the program
        std::string buildOptions;
        std::string includePath = NuiOpenCLGlobal::instance().kernelDir();

#ifdef __APPLE__
        // Apple's OpenCL Compiler doesn't support -I "path" syntax.
        openclutil::replaceSubStrs(includePath, " " , "\\ ");
#else
        std::string::size_type pos = includePath.find(" ");
        if( pos != std::string::npos ) {
            includePath = "\"" + includePath + "\"";
        }
#endif
        includePath = "-I " + includePath;

        // Supress warnings, this is annoying
        buildOptions = includePath + " -w";

        err = clBuildProgram(program, 1, &device, buildOptions.c_str(), NULL, NULL);
        NUI_CHECK_CL_ERR_STREAM(err, errStream);

        // Check build status
        cl_build_status buildStatus;
        err = clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_STATUS, sizeof(cl_build_status), &buildStatus, NULL);
        NUI_CHECK_CL_ERR_STREAM(err, errStream);

        if (buildStatus != CL_BUILD_SUCCESS)
        {
            size_t buildLogSize = 0;
            err = clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, NULL, &buildLogSize);
            NUI_CHECK_CL_ERR_STREAM_RET_NULLPTR(err, errStream);

            std::vector<char> buildLog(buildLogSize + 1);
            err = clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, buildLogSize, &buildLog[0], NULL);
            NUI_CHECK_CL_ERR_STREAM_RET_NULLPTR(err, errStream);

            buildLog[buildLogSize] = '\0';

            errStream
                << std::endl << std::endl
                << "------------start build log------------"
                << std::endl << std::endl
                << (const char*)&buildLog[0]
                << std::endl << std::endl
                << "-------------end build log-------------"
                << std::endl << std::endl;

            errStream
                << std::endl << std::endl
                << "------------start program source------------"
                << std::endl << std::endl
                << source
                << std::endl << std::endl
                << "-------------end program source-------------"
                << std::endl << std::endl;

            err = clReleaseProgram(program);
            NUI_CHECK_CL_ERR_STREAM_RET_NULLPTR(err, errStream);

            return nullptr;
        }

        return buildCLKernelFromProgram(kernelName, program, errStream);
    }

    std::shared_ptr<OpenCLKernelInfo> buildCLKernelFromFile(
        const std::string&    kernelName,
        const std::string&    programFileName,
        std::stringstream&    errStream
        )
    {
        // Read as string
        std::string programText;
        {
            std::ifstream inFile;
            std::string filename = programFileName;
            inFile.open(filename.c_str());
            if (!inFile.good()){
                const std::string& kernelDir = NuiOpenCLGlobal::instance().kernelDir();
                filename = kernelDir + std::string("/") + filename;
                inFile.open(filename.c_str());
            }
            assert(inFile.good());
            if (inFile.good()){
                std::stringstream buffer;
                buffer << inFile.rdbuf();
                programText = buffer.str();
                inFile.close();
            }
            else {
                errStream << "Failed to find kernel file: '"<<filename<<"'"<<std::endl;
                return nullptr;
            }
        }

        return buildCLKernelFromSource(kernelName, programText.c_str(), errStream);
    }

    size_t computeGlobalWorkSize(const size_t elementCount, const size_t localWorkSize)
    {
        const size_t remain = elementCount % localWorkSize;
        if (remain)
            return elementCount + (localWorkSize - remain);
        else
            return elementCount;
    }

    cl_mem createCLObjectFromHWBuffer(
        cl_mem_flags flags, void* bufferResHandle)
    {
        cl_int err = CL_SUCCESS;
        cl_mem mem = NULL;
        cl_context context = NuiOpenCLGlobal::instance().clContext();

        if (NuiOpenCLGlobal::instance().isGL())
        {
            mem = clCreateFromGLBuffer(
                context,
                flags,
                *((GLuint*)bufferResHandle),
                &err);
            NUI_CHECK_CL_ERR(err);
        }
#ifdef _WIN32
        else if (isUsingNVOpenCLDevice() && clCreateFromD3D11BufferNV)
        {
            mem = clCreateFromD3D11BufferNV(
                context,
                flags,
                (ID3D11Buffer*)bufferResHandle,
                &err);
            NUI_CHECK_CL_ERR(err);
        }
        else if (clCreateFromD3D11BufferKHR)
        {
            mem = clCreateFromD3D11BufferKHR(
                context,
                flags,
                (ID3D11Buffer*)bufferResHandle,
                &err);
            NUI_CHECK_CL_ERR(err);
        }
#endif

        return mem;
    }

    cl_mem createCLObjectFromHWTexture3D(cl_mem_flags flags, void* textureResHandle)
    {
        cl_int err = CL_SUCCESS;
        cl_mem mem = NULL;
        cl_context context = NuiOpenCLGlobal::instance().clContext();

        if (NuiOpenCLGlobal::instance().isGL())
        {
            mem = clCreateFromGLTexture3D(
                context,
                flags,
                GL_TEXTURE_3D,
                0,
                *((GLuint*)textureResHandle),
                &err);
            NUI_CHECK_CL_ERR(err);
        }
#ifdef _WIN32
        else if (isUsingNVOpenCLDevice() && clCreateFromD3D11Texture3DNV)
        {
            mem = clCreateFromD3D11Texture3DNV(
                context,
                flags,
                (ID3D11Texture3D*)textureResHandle,
                0,
                &err);
            NUI_CHECK_CL_ERR(err);
        }
        else if (clCreateFromD3D11Texture3DKHR)
        {
            mem = clCreateFromD3D11Texture3DKHR(
                context,
                flags,
                (ID3D11Texture3D*)textureResHandle,
                0,
                &err);
            NUI_CHECK_CL_ERR(err);
        }
#endif

        return mem;
    }

	cl_mem createCLObjectFromHWTexture2D(cl_mem_flags flags, void* textureResHandle)
	{
		cl_int err = CL_SUCCESS;
		cl_mem mem = NULL;
		cl_context context = NuiOpenCLGlobal::instance().clContext();

		if (NuiOpenCLGlobal::instance().isGL())
		{
			mem = clCreateFromGLTexture2D(
				context,
				flags,
				GL_TEXTURE_2D,
				0,
				*((GLuint*)textureResHandle),
				&err);
			NUI_CHECK_CL_ERR(err);
		}
#ifdef _WIN32
		else if (isUsingNVOpenCLDevice() && clCreateFromD3D11Texture2DNV)
		{
			mem = clCreateFromD3D11Texture2DNV(
				context,
				flags,
				(ID3D11Texture2D*)textureResHandle,
				0,
				&err);
			NUI_CHECK_CL_ERR(err);
		}
		else if (clCreateFromD3D11Texture2DKHR)
		{
			mem = clCreateFromD3D11Texture2DKHR(
				context,
				flags,
				(ID3D11Texture2D*)textureResHandle,
				0,
				&err);
			NUI_CHECK_CL_ERR(err);
		}
#endif

		return mem;
	}

    // Lock a GL/DX object for OpenCL computation
    void enqueueAcquireHWObjects(cl_uint num_objects, const cl_mem* mem_objects,
        cl_uint num_events_in_wait_list, const cl_event* event_wait_list, cl_event* event)
    {
        cl_int err = CL_SUCCESS;
        cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

        if (NuiOpenCLGlobal::instance().isGL())
        {
            // According to the spec, all GL operation should be done prior to 'clEnqueueAcquireGLObjects'
            NuiOpenGLThread::instance().enqueueAndWait([&]()
            {
                glFinish();
                err = clEnqueueAcquireGLObjects(queue, num_objects, mem_objects,
                    num_events_in_wait_list, event_wait_list, event);
                NUI_CHECK_CL_ERR(err);
            });
        }
#ifdef _WIN32
        else if (isUsingNVOpenCLDevice() && clEnqueueAcquireD3D11ObjectsNV)
        {
            err = clEnqueueAcquireD3D11ObjectsNV(queue, num_objects, const_cast<cl_mem*>(mem_objects),
                num_events_in_wait_list, event_wait_list, event);
            NUI_CHECK_CL_ERR(err);
        }
        else if (clEnqueueAcquireD3D11ObjectsKHR)
        {
            err = clEnqueueAcquireD3D11ObjectsKHR(queue, num_objects, mem_objects,
                num_events_in_wait_list, event_wait_list, event);
            NUI_CHECK_CL_ERR(err);
        }
#endif
    }

    // Unlock a GL/DX object for OpenCL computation
    void enqueueReleaseHWObjects(cl_uint num_objects, const cl_mem* mem_objects,
        cl_uint num_events_in_wait_list, const cl_event* event_wait_list, cl_event* event)
    {
        cl_int err = CL_SUCCESS;
        cl_command_queue queue = NuiOpenCLGlobal::instance().clQueue();

        if (NuiOpenCLGlobal::instance().isGL())
        {
            NuiOpenGLThread::instance().enqueueAndWait([&]()
            {
                err = clEnqueueReleaseGLObjects(queue, num_objects, mem_objects,
                    num_events_in_wait_list, event_wait_list, event);
                NUI_CHECK_CL_ERR(err);
            });
        }
#ifdef _WIN32
        else if (isUsingNVOpenCLDevice() && clEnqueueReleaseD3D11ObjectsNV)
        {
            err = clEnqueueReleaseD3D11ObjectsNV(queue, num_objects, const_cast<cl_mem*>(mem_objects),
                num_events_in_wait_list, event_wait_list, event);
            NUI_CHECK_CL_ERR(err);
        }
        else if (clEnqueueReleaseD3D11ObjectsKHR)
        {
            err = clEnqueueReleaseD3D11ObjectsKHR(queue, num_objects, mem_objects,
                num_events_in_wait_list, event_wait_list, event);
            NUI_CHECK_CL_ERR(err);
        }
#endif
        // According to spec, we need to wait the event generate by clEnqueueRelease* before
        // any GL/DX accessing
        if (event) {
            err = clWaitForEvents(1, event);
            NUI_CHECK_CL_ERR(err);
        }
    }

    unsigned int getMemObjectSizeCL(cl_mem b)
    {
        if (!b)
            return 0;

        size_t s = 0;
        cl_int err = clGetMemObjectInfo(b, CL_MEM_SIZE, sizeof(size_t), (void*)(&s), nullptr);
        NUI_CHECK_CL_ERR(err);
        return (unsigned int)s;
    }

    void releaseEvents(cl_uint num_events, cl_event* event_list)
    {
        assert(event_list);
        for (cl_uint i = 0; i < num_events; i++)
        {
            cl_int err = clReleaseEvent(event_list[i]);
            NUI_CHECK_CL_ERR(err);
        }
    }

    void releaseEvents(std::vector<cl_event>& events)
    {
        if (!events.empty())
            releaseEvents((cl_uint)events.size(), events.data());
    }

    unsigned int estimateImageCLSize(const cl_image_format *f, size_t w, size_t h, size_t d)
    {
        unsigned int elementSizeInByte = 1;
        {
            if (f) {
                unsigned int channel_count = 1;
                {
                    switch (f->image_channel_order)
                    {
                    case CL_R:
                    case CL_Rx:
                    case CL_A:
                    case CL_INTENSITY:
                    case CL_LUMINANCE:
                    {
                        channel_count = 1; break;
                    }
                    case CL_RG:
                    case CL_RGx:
                    case CL_RA:
                    {
                        channel_count = 2; break;
                    }
                    case CL_RGB:
                    case CL_RGBx:
                    {
                        channel_count = 3; break;
                    }
                    case CL_RGBA:
                    case CL_ARGB:
                    case CL_BGRA:
                    {
                        channel_count = 4; break;
                    }
                    default:
                    {
                        NUI_WARN("Unrecognized opencl image channel order(%d)!\n", (int)(f->image_channel_order));
                    }
                    };
                }

                unsigned int channel_size = 1;
                {
                    switch (f->image_channel_data_type)
                    {
                    case CL_SNORM_INT8:
                    case CL_UNORM_INT8:
                    case CL_SIGNED_INT8:
                    case CL_UNSIGNED_INT8:
                    {
                        channel_size = 1; break;
                    }
                    case CL_SNORM_INT16:
                    case CL_UNORM_INT16:
                    case CL_SIGNED_INT16:
                    case CL_UNSIGNED_INT16:
                    case CL_UNORM_SHORT_565:
                    case CL_UNORM_SHORT_555:
                    case CL_HALF_FLOAT:
                    {
                        channel_size = 2; break;
                    }
                    case CL_UNORM_INT_101010:
                    case CL_SIGNED_INT32:
                    case CL_UNSIGNED_INT32:
                    case CL_FLOAT:
                    {
                        channel_size = 4; break;
                    }
                    default:
                    {
                        NUI_WARN("Unrecognized opencl image channel data type(%d)!\n", (int)(f->image_channel_data_type));
                    }
                    };
                }
                elementSizeInByte = channel_count*channel_size;
            }
        }

        return (unsigned int)(elementSizeInByte*w*h*d);
    }

    // Helper function to set kernel args
    bool setKernelArgs(
        cl_kernel kernel,
        const std::vector< std::pair<size_t, void*> >& args)
    {
        bool ret = true;
        cl_int err = CL_SUCCESS;
        for (auto i = 0; i < args.size(); ++i)
        {
            err = clSetKernelArg(kernel, i, args[i].first, args[i].second);
            ret = ret && (err == CL_SUCCESS);
            NUI_CHECK_CL_ERR(err);
        }
        return ret;
    }

} // namespace openclutil
