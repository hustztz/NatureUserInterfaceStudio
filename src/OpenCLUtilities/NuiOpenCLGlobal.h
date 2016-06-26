#pragma once

#include <clew/clew.h>
#include <string>

// A singleton class to hold global unique data for sculpt core

class NuiOpenCLGlobal
{
public:
    static NuiOpenCLGlobal& instance();

    // This function should cleanup any resource hold inside this class
    // that needs explicit release like opencl queue.
    // It should be called during plugin uninitializing.
    void cleanup();

    // Functions to get the opencl objects
    cl_context       clContext()const  { return _cl_context; }
    cl_device_id     clDeviceId()const { return _cl_device; }
    cl_command_queue clQueue()const    { return _cl_queue; }

    // Setup opencl objects, should only be called once during plugin initializing
    bool setupOpenCL(cl_context c, cl_device_id d, cl_command_queue q);

	bool initializeOpenCL();

    // Whether opencl objects are ready
    bool isCLReady() const { return _cl_context && _cl_device && _cl_queue; }

	void uninitialize();

    // Whether it is using opengl renderer
    bool isGL()const { return _isGL; }
    void isGL(bool b) { _isGL = b; }

    // Kernel file directory
    const std::string& kernelDir()const { return _kernel_dir; }
    void kernelDir(const std::string& d) { _kernel_dir = d; }

    // Temp debug directory
    const std::string& tmpDir()const { return _tmp_dir; }
    std::string prepareTmpDir(const std::string& tmpDirRoot);

    // Output something to a file under temp directory
    std::string dumpFileToTmpDir(const std::string& filename, const std::string& filecontent) const;

    // Whether force rebuild all kernels
    bool kernelDevMode() const { return _kernelDevMode; }
    void kernelDevMode(bool b) { _kernelDevMode = b; }

private:
    cl_context       _cl_context;
    cl_device_id     _cl_device;
    cl_command_queue _cl_queue;
    bool _isGL;
    std::string      _kernel_dir;
    std::string      _tmp_dir;
    bool _kernelDevMode;

private:
    // privates these for singleton mechanism
    NuiOpenCLGlobal();
    ~NuiOpenCLGlobal();
    NuiOpenCLGlobal(const NuiOpenCLGlobal&){}
    NuiOpenCLGlobal& operator=(const NuiOpenCLGlobal&){ return *this; }
};


