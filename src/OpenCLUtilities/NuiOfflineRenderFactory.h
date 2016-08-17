#include "NuiMappable.h"

typedef void (*FN_initializeOfflineRender)();
typedef void (*FN_runOfflineRender)(NuiMappable4f&, NuiMappablef&, NuiMappablef&, int, float, float);

class NuiOfflineRenderFactory
{
public:
	static void initializeOfflineRender();
	static void runOfflineRender(NuiMappable4f& vb, NuiMappablef& rbMin, NuiMappablef& rbMax, int size, float sensorDepthMin, float sensorDepthMax);

public:
	static void RegisterInitializeOfflineRenderFn(FN_initializeOfflineRender f)
	{
		pInitializeOfflineRenderFn = f;
	}
	static void RegisterRunOfflineRenderFn(FN_runOfflineRender f)
	{
		pRunOfflineRenderFn = f;
	}
	
private:
	static FN_initializeOfflineRender pInitializeOfflineRenderFn;
	static FN_runOfflineRender pRunOfflineRenderFn;
};