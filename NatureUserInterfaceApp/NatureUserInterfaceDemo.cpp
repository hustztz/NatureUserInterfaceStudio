// NatureUserInterfaceDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "GUI/NuiGuiController.h"

int _tmain(int argc, _TCHAR* argv[])
{
	bool bIsDrawDepth = false;
	bool bHasFace = false;
	if(argc == 2)
	{
		wchar_t strIsDepth[] = L"-d";
		wchar_t strHasFace[] = L"-f";
		if (wcscmp(argv[1],strIsDepth) == 0)
		{
			bIsDrawDepth = true;
		}
		else if (wcscmp(argv[1],strHasFace) == 0)
		{
			bHasFace = true;
		}
	}

	NuiGuiController mainController;
	mainController.launch();

	return 0;
}

