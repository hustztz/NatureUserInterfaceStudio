#pragma once

#include <map>
#include <string>

typedef int(*eventBeginFuncPtr)(int, int, const char*, const char*);
typedef void(*eventEndFuncPtr)(int);

class NuiProfilingScope
{
public:
    static eventBeginFuncPtr eventBeginFunc;
    static eventEndFuncPtr eventEndFunc;
    static int sNuiCategory;

    NuiProfilingScope(
        const char* eventName,
        const char* description = nullptr);

    ~NuiProfilingScope()
    {
        if (eventEndFunc && _eventId != -1)
        {
            eventEndFunc(_eventId);
        }
    }

private:
    static int getColorIndex(const char* str);
    static std::map<std::string, int> _colorMap;
    static const int sMaxColorIndex = 14;
    int _eventId;
};

