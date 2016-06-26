#pragma once

class NuiMayaProfiler
{
public:
	static void assignFuncPtr();
	static void addNuiCategory(const char*);
	static int NuiCategory;
private:
	NuiMayaProfiler();
	~NuiMayaProfiler();
};