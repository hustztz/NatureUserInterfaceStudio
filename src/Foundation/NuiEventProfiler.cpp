#include "NuiEventProfiler.h"

#include <stdio.h>

addCategoryFuncPtr NuiEventProfiler::addCategoryFunc = NULL;
eventBeginFuncPtr NuiEventProfiler::eventBeginFunc = NULL;
eventEndFuncPtr NuiEventProfiler::eventEndFunc = NULL;

int NuiEventProfiler::sNuiCategory = -1;