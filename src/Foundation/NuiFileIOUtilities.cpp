#include "NuiFileIOUtilities.h"

#include "NuiTimeStamp.h"

#include <fstream>

namespace NuiFileIOUtilities
{
	bool writeFrameImage (const std::string& fileName, UINT nWidth, UINT nHeight, const char* pBuffer, UINT bufferSize)
	{
		// Open file in binary appendable
		std::ofstream fpout (fileName.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
		if( !fpout.is_open() )
		{
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		fpout.write (reinterpret_cast<const char*> (&nWidth), sizeof (UINT));
		fpout.write (reinterpret_cast<const char*> (&nHeight), sizeof (UINT));

		fpout.write (pBuffer, bufferSize);

		// Close file
		fpout.close ();
		return true;
	}

	bool readFrameImageHeader (const std::string& fileName, UINT* pWidth, UINT* pHeight)
	{
		// Open file in binary appendable
		std::ifstream fpin (fileName.c_str(), std::ios::in | std::ios::binary);
		if( !fpin.is_open() )
		{
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		if (pWidth)
			fpin.read (reinterpret_cast<char*> (pWidth), sizeof (UINT));
		if (pHeight)
			fpin.read (reinterpret_cast<char*> (pHeight), sizeof (UINT));

		// Close file
		fpin.close ();
		return true;
	}

	bool readFrameImageBuffer (const std::string& fileName, char* pBuffer, UINT bufferSize)
	{
		// Open file in binary appendable
		std::ifstream fpin (fileName.c_str(), std::ios::in | std::ios::binary);
		if( !fpin.is_open() )
		{
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		fpin.seekg(2 * sizeof(UINT), std::ios::beg);
		if (pBuffer && bufferSize > 0)
			fpin.read (pBuffer, bufferSize);

		// Close file
		fpin.close ();
		return true;
	}

	bool		writeCamera (const std::string& fileName, float intr_fx, float intr_fy, float intr_cx, float intr_cy)
	{
		// Open file in binary appendable
		std::ofstream fpout (fileName.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
		if( !fpout.is_open() )
		{
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		fpout.write (reinterpret_cast<const char*> (&intr_fx), sizeof (float));
		fpout.write (reinterpret_cast<const char*> (&intr_fy), sizeof (float));
		fpout.write (reinterpret_cast<const char*> (&intr_cx), sizeof (float));
		fpout.write (reinterpret_cast<const char*> (&intr_cy), sizeof (float));

		// Close file
		fpout.close ();
		return true;
	}

	bool		readCamera (const std::string& fileName, float* intr_fx, float* intr_fy, float* intr_cx, float* intr_cy)
	{
		if(!intr_fx || !intr_fy)
			return false;

		// Open file in binary appendable
		std::ifstream fpin (fileName.c_str(), std::ios::in | std::ios::binary);
		if( !fpin.is_open() )
		{
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		if (intr_fx)
			fpin.read (reinterpret_cast<char*> (intr_fx), sizeof (float));
		if (intr_fy)
			fpin.read (reinterpret_cast<char*> (intr_fy), sizeof (float));
		if (intr_cx)
			fpin.read (reinterpret_cast<char*> (intr_cx), sizeof (float));
		if (intr_cy)
			fpin.read (reinterpret_cast<char*> (intr_cy), sizeof (float));

		// Close file
		fpin.close ();
		return true;
	}

	bool		writeTime (const std::string& fileName, const std::map<std::string, NuiTimeStamp>& timeMap, double timeFreq)
	{
		// Open file
		std::ofstream fpout (fileName.c_str(), std::ios::out | std::ios::app);
		if( !fpout.is_open() )
		{
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		std::map<std::string, NuiTimeStamp>::const_iterator iter;
		for(iter = timeMap.begin();iter != timeMap.end(); ++iter)
		{
			
			fpout << iter->first << " timeStamp:" << double(iter->second.m_sumTime) / (timeFreq + 0.05) << std::endl;
			fpout << iter->first << " count:" << iter->second.m_count << std::endl;
			fpout << iter->first << " fps:" << timeFreq * iter->second.m_count / (double(iter->second.m_sumTime) + 0.05) << std::endl;
		}
		
		// Close file
		fpout.close ();
		return true;
	}
}