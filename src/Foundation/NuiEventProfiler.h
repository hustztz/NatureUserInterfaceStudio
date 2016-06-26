#pragma once

typedef int (*addCategoryFuncPtr)(const char*);
typedef int (*eventBeginFuncPtr)(int, int, const char*, const char*);
typedef void (*eventEndFuncPtr)(int);

class NuiEventProfiler
{
public:
	static addCategoryFuncPtr addCategoryFunc;
	static eventBeginFuncPtr eventBeginFunc;
	static eventEndFuncPtr eventEndFunc;
	static int sNuiCategory;
private:
	NuiEventProfiler();
	~NuiEventProfiler();
};