#pragma once

#include "stdafx.h"

#include <map>
#include <string>

struct NuiTimeStamp;

namespace NuiFileIOUtilities
{
	bool		writeFrameImage (const std::string& fileName, bool bCompressed, INT64 timeStamp, UINT nWidth, UINT nHeight, const char* pBuffer, UINT bufferSize);
	bool		readFrameImageHeader (const std::string& fileName, INT64* timeStamp, UINT* pWidth, UINT* pHeight);
	bool		readFrameImageBuffer (const std::string& fileName, char* pBuffer, UINT bufferSize);

	bool		writeCamera (const std::string& fileName, float intr_fx, float intr_fy, float intr_cx, float intr_cy);
	bool		readCamera (const std::string& fileName, float* intr_fx, float* intr_fy, float* intr_cx, float* intr_cy);

	bool		writeTimeLog (const std::string& fileName, const std::map<std::string, NuiTimeStamp>& timeMap);
	bool		writeDayTime (const std::string& fileName);
}