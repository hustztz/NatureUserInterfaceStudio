#include "NuiProfilingScope.h"

eventBeginFuncPtr NuiProfilingScope::eventBeginFunc = nullptr;
eventEndFuncPtr NuiProfilingScope::eventEndFunc = nullptr;
int NuiProfilingScope::sNuiCategory = -1;
std::map<std::string, int> NuiProfilingScope::_colorMap;

NuiProfilingScope::NuiProfilingScope(
    const char* eventName,
    const char* description/* = nullptr*/)
{
    if (eventBeginFunc)
    {
        int colorIndex = NuiProfilingScope::getColorIndex(eventName);
        _eventId = eventBeginFunc(sNuiCategory, colorIndex, eventName, description);
    }
}

int NuiProfilingScope::getColorIndex(const char* str)
{
    auto cIt = _colorMap.find(str);
    if (cIt != _colorMap.end()){
        return cIt->second;
    }

    static int index = 0;
    int newColIdx = (index++) % (sMaxColorIndex + 1);
    _colorMap.insert(std::make_pair(std::string(str), newColIdx));
    return newColIdx;
}

