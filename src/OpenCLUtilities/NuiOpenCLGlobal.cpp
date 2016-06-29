// =======================================================================
// Copyright 2015 Autodesk, Inc. All rights reserved.
//
// This computer source code and related instructions and comments are the
// unpublished confidential  and proprietary information of Autodesk, Inc.
// and are protected under applicable copyright and trade secret law. They 
// may not be disclosed to, copied  or used by any third party without the 
// prior written consent of Autodesk, Inc.
// =======================================================================

#include "NuiOpenCLGlobal.h"

#include <assert.h>
#include <sstream>
#include <fstream>
#include <ctime>

#include "Foundation/NuiDebugMacro.h"
#include "NuiOpenCLUtil.h"
#include "NuiOpenCLKernelManager.h"

#ifdef _WIN32
#include <windows.h>
#endif

NuiOpenCLGlobal& NuiOpenCLGlobal::instance() {
    static NuiOpenCLGlobal g;
	return g;
}


NuiOpenCLGlobal::NuiOpenCLGlobal():
_cl_context(nullptr),
_cl_device(nullptr),
_cl_queue(nullptr),
_isGL(true),
_kernel_dir(""),
_tmp_dir(""),
_kernelDevMode(false)
{
}

NuiOpenCLGlobal::~NuiOpenCLGlobal()
{}

void NuiOpenCLGlobal::cleanup()
{
    // Release opencl command queue
    if (_cl_queue) {
        cl_int err = clReleaseCommandQueue(_cl_queue);
        NUI_CHECK_CL_ERR(err);
        _cl_queue = nullptr;
    }
}

bool NuiOpenCLGlobal::setupOpenCL(cl_context c, cl_device_id d, cl_command_queue q)
{
    assert(c);
    assert(d);
    assert(q);

    _cl_context = c;
    _cl_device  = d;

    if (_cl_queue) {
        cl_int err = clReleaseCommandQueue(_cl_queue);
        NUI_CHECK_CL_ERR(err);
        _cl_queue = nullptr;
    }
    _cl_queue = q;

    return isCLReady();
}

bool NuiOpenCLGlobal::initializeOpenCL()
{
	bool printDebugInfo = true;

	int clewStatus = clewInit("OpenCL.dll");

	// Kernel dev mode, when it is on, it will rebuild kernels at certain time
	// like when tool context activated. Using this is to facilitate the kernel develop
	// work, we don't need to restart or even rebuild maya to make new kernel change
	// take effect.
	// Of source, rebuilding kernel too often may have performance penalty, so please
	// use it when during kernel development work only.
	{
		_kernelDevMode = true;
		if (_kernelDevMode) {
			printf("Kernel develop mode is on.\n");
		}
		else {
			printf("Kernel develop mode is off.\n");
		}
	}

	cl_int ciErrNum;
	cl_uint numPlatforms = 1;
	ciErrNum = clGetPlatformIDs(4, NULL, &numPlatforms);
	//CL_OK(ciErrNum);
	cl_platform_id* clPlatformIDs = (cl_platform_id*)malloc(numPlatforms * sizeof(cl_platform_id));
	ciErrNum = clGetPlatformIDs(numPlatforms, clPlatformIDs, NULL);
	//CL_OK(ciErrNum);
	if (printDebugInfo) printf("There are %d openCL platforms in your pc.\n", numPlatforms);
	// Search for the platforms the user wants
	for(unsigned int i=0; i<numPlatforms; i++)
	{
		char platformName[128];
		char vendor[128];
		char version[128];

		// print out what the platform is
		ciErrNum = clGetPlatformInfo(clPlatformIDs[i], CL_PLATFORM_VENDOR, 128, vendor, NULL);
		//CL_OK(ciErrNum);
		ciErrNum = clGetPlatformInfo(clPlatformIDs[i], CL_PLATFORM_NAME, 128, platformName, NULL);
		//CL_OK(ciErrNum);
		ciErrNum = clGetPlatformInfo(clPlatformIDs[i], CL_PLATFORM_VERSION, 128, version, NULL);
		//CL_OK(ciErrNum);

		if (printDebugInfo) printf(" %d: %s. %s. %s.\n", i, vendor, platformName, version);

		// print out the supported extensions
		char extensions[1024];
		ciErrNum = clGetPlatformInfo(clPlatformIDs[i], CL_PLATFORM_EXTENSIONS, 1024, extensions, NULL);
		if (printDebugInfo) printf(" Supported extensions: %s\n", extensions);
	}

	cl_uint numDevices;
	ciErrNum = clGetDeviceIDs(clPlatformIDs[0], CL_DEVICE_TYPE_GPU, 1, NULL, &numDevices);
	//CL_OK(ciErrNum);
	cl_device_id* clDeviceIDs = (cl_device_id*)malloc(numDevices * sizeof(cl_device_id));
	ciErrNum = clGetDeviceIDs(clPlatformIDs[0], CL_DEVICE_TYPE_GPU, numDevices, clDeviceIDs, NULL);
	//CL_OK(ciErrNum);
	if (printDebugInfo) printf("There are %d openCL devices in your platform #0.\n", numDevices);
	// search for a GPU device
	for(unsigned int i=0; i<numDevices; i++)
	{
		// make sure the device is a GPU device
		char deviceName[128];
		ciErrNum = clGetDeviceInfo(clDeviceIDs[i], CL_DEVICE_NAME, 128, deviceName, NULL);
		//CL_OK(ciErrNum);
		cl_ulong memSize;
		ciErrNum = clGetDeviceInfo(clDeviceIDs[i], CL_DEVICE_GLOBAL_MEM_CACHE_SIZE, sizeof(cl_ulong), &memSize, NULL);
		//CL_OK(ciErrNum);
		cl_uint addr_data;
		ciErrNum = clGetDeviceInfo(clDeviceIDs[i], CL_DEVICE_ADDRESS_BITS, sizeof(cl_uint), &addr_data, NULL);
		//CL_OK(ciErrNum);

		if (printDebugInfo) printf(" %d: %s. %l. %d.\n", i, deviceName, memSize, addr_data);
	}

	cl_context_properties props[16];  // cl_context_properties is a null-terminated array of pairs.  property id, value.
	unsigned int numProps = 0;
	_isGL = true;
	if(_isGL)
	{
		// GL interop uses clCreateContext to chose the gl device association.
#ifdef _WIN32
		// commented out code works
		props[numProps++] = CL_GL_CONTEXT_KHR;      props[numProps++] = (cl_context_properties) wglGetCurrentContext();
		props[numProps++] = CL_WGL_HDC_KHR;         props[numProps++] = (cl_context_properties) wglGetCurrentDC();
		props[numProps++] = CL_CONTEXT_PLATFORM;    props[numProps++] = (cl_context_properties) clPlatformIDs[0];
#endif
		props[numProps++] = 0;
	}
	cl_context clContext = clCreateContext(props, 1, &clDeviceIDs[0], NULL, NULL, &ciErrNum);


	bool needCLProfiling = false;
#ifdef _DEBUG
	needCLProfiling = true;
	if(needCLProfiling)
		printf("OpenCL evaluator has OpenCL profiling on the command queue enabled.  This feature has been observed to cause a steady, progressive performance degradation.\n");
#endif
	cl_command_queue clCommandQueue = clCreateCommandQueue(clContext, clDeviceIDs[0], needCLProfiling ? CL_QUEUE_PROFILING_ENABLE : 0, &ciErrNum);

	free(clPlatformIDs);

	// init globals to core
	if (clContext) {
		setupOpenCL(
			clContext,
			clDeviceIDs[0],
			clCommandQueue);

		if (isCLReady())
		{
			printf("OpenCL initialize successfully.\n");
		}
		else {
			printf("OpenCL initialize failed!!\n");
			return false;
		}

		// Set kernel dir
		std::string kernelsFolder = getenv("NUI_LOCATION") ? getenv("NUI_LOCATION") : "\\src";
		kernelsFolder += "\\Kernels";
		kernelDir(kernelsFolder.c_str());

		// Set temp dir
		{
			std::string tmpDir = prepareTmpDir(kernelsFolder.c_str());
			printf("TempDir:  %s\n",
				(tmpDir != "") ? tmpDir.c_str() :
				"Failed to create!");
		}

		// Only build all used opencl kernels once ahead of time here
		// when in debug mode.
		// Set whether debug mode
		NuiOpenCLKernelManager::instance().buildAllShaders();
	}
	else {
		printf("OpenCL is not available, sculpting will not work!\n");
		return false;
	}

	return true;
}


void NuiOpenCLGlobal::uninitialize()
{
	NuiOpenCLKernelManager::instance().reset();
	NuiOpenCLGlobal::instance().cleanup();
}

std::string NuiOpenCLGlobal::prepareTmpDir(const std::string& tmpDirRoot)
{
    std::stringstream ss;
    ss << tmpDirRoot << "/_____nui/";

    // Append time stamp
    {
        time_t t = time(nullptr);
        tm* curTime = localtime(&t);
        if (curTime)
            ss << 1900 + curTime->tm_year << 1 + curTime->tm_mon << curTime->tm_mday;
    }

    const std::string finalTmpDirName = ss.str();

    /*if (xgutil::createDir(finalTmpDirName+"/", false)) {
        _tmp_dir = finalTmpDirName;
        return _tmp_dir;
    }
    else*/ {
        _tmp_dir = tmpDirRoot;
        return _tmp_dir;
    }
}

// Output something to a file under temp directory
std::string NuiOpenCLGlobal::dumpFileToTmpDir(
    const std::string& filename, const std::string& filecontent) const
{
    const std::string& tempDir = tmpDir();
    assert(tempDir != "");
    if (tempDir == "")
        return "";

    const std::string absFileName = tempDir + "/" + filename;

    std::ofstream ofile(absFileName);
    ofile << filecontent;

    return absFileName;
}



