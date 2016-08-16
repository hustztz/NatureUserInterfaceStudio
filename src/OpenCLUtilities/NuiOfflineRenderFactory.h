#include <clew/clew.h>

typedef bool (*FN_initializeOfflineRender)(unsigned int);
typedef void (*FN_uninitializeOfflineRender)();
typedef void (*FN_runOfflineRender)(cl_mem);

class NuiOfflineRenderFactory
{
public:
	static bool initializeOfflineRender(unsigned int vbId);
	static void uninitializeOfflineRender();
	static void runOfflineRender(cl_mem buffer);

public:
	static void RegisterInitializeOfflineRenderFn(FN_initializeOfflineRender f)
	{
		pInitializeOfflineRenderFn = f;
	}
	static void RegisterUninitializeOfflineRenderFn(FN_uninitializeOfflineRender f)
	{
		pUninitializeOfflineRenderFn = f;
	}
	static void RegisterAsColor4fBufferCLFn(FN_runOfflineRender f)
	{
		pRunOfflineRenderFn = f;
	}
	
private:
	static FN_initializeOfflineRender pInitializeOfflineRenderFn;
	static FN_uninitializeOfflineRender pUninitializeOfflineRenderFn;
	static FN_runOfflineRender pRunOfflineRenderFn;
};