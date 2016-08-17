#include "NuiOfflineRenderFactory.h"
#include <assert.h>

FN_initializeOfflineRender NuiOfflineRenderFactory::pInitializeOfflineRenderFn = nullptr;
FN_runOfflineRender NuiOfflineRenderFactory::pRunOfflineRenderFn = nullptr;

void NuiOfflineRenderFactory::initializeOfflineRender()
{
	assert(pInitializeOfflineRenderFn);
	if (!pInitializeOfflineRenderFn) {
		return;
	}
	pInitializeOfflineRenderFn();
}

void NuiOfflineRenderFactory::runOfflineRender(NuiMappable4f& vb, NuiMappablef& rbMin, NuiMappablef& rbMax, int size, float sensorDepthMin, float sensorDepthMax)
{
	assert(pRunOfflineRenderFn);
	if (!pRunOfflineRenderFn) {
		return;
	}
	pRunOfflineRenderFn(vb, rbMin, rbMax, size, sensorDepthMin, sensorDepthMax);
}