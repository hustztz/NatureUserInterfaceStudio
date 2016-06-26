#include "NuiMayaProfiler.h"
#include "Foundation/NuiEventProfiler.h"
#include "Foundation/NuiProfilingScope.h"

#include <maya/MProfiler.h>

int NuiMayaProfiler::NuiCategory = -1;

int eventBegin(int categoryId, int colorIndex, const char* eventName, const char* description)
{
	MProfiler::ProfilingColor profilerColorIndex = MProfiler::ProfilingColor(colorIndex);
	return MProfiler::eventBegin(categoryId, profilerColorIndex, eventName, description);
}

void NuiMayaProfiler::assignFuncPtr()
{
	NuiEventProfiler::addCategoryFunc = &MProfiler::addCategory;
	NuiEventProfiler::eventBeginFunc = &eventBegin;
	NuiEventProfiler::eventEndFunc = &MProfiler::eventEnd;

	NuiProfilingScope::sNuiCategory = MProfiler::addCategory("NatureUserInterface");
	NuiProfilingScope::eventBeginFunc = eventBegin;
	NuiProfilingScope::eventEndFunc = &MProfiler::eventEnd;
}

void NuiMayaProfiler::addNuiCategory(const char* categoryName)
{
	NuiCategory = MProfiler::addCategory(categoryName);
	NuiEventProfiler::sNuiCategory = NuiCategory;
}