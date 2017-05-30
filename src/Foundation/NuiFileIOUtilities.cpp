#include "NuiFileIOUtilities.h"

#include "NuiTimeStamp.h"
#include "NuiLogger.h"

#include <fstream>
#include <string>
#include <ctime>
#include <zlib.h>

namespace NuiFileIOUtilities
{
	bool writeFrameImage (const std::string& fileName, bool bCompressed, INT64 timeStamp, UINT nWidth, UINT nHeight, const char* pBuffer, UINT bufferSize)
	{
		// Open file in binary appendable
		std::ofstream fpout (fileName.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);
		if( !fpout.is_open() )
		{
			LOG4CPLUS_FATAL(NuiLogger::instance().consoleLogger(), ("could not open file." + fileName).c_str());
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		fpout.write(reinterpret_cast<const char*> (&timeStamp), sizeof(INT64));
		fpout.write (reinterpret_cast<const char*> (&nWidth), sizeof (UINT));
		fpout.write (reinterpret_cast<const char*> (&nHeight), sizeof (UINT));

		if (bCompressed)
		{
			UINT compressedBufferSize = compressBound(bufferSize);
			Bytef* compressedBuffer = (Bytef*)malloc(sizeof(Bytef) * compressedBufferSize);
			if (Z_OK == compress2(compressedBuffer, (unsigned long *)&compressedBufferSize, (const Bytef*)pBuffer, bufferSize, Z_BEST_SPEED))
			{
				fpout.write(reinterpret_cast<const char*> (&compressedBufferSize), sizeof(UINT));
				fpout.write(reinterpret_cast<const char*> (compressedBuffer), compressedBufferSize);
			}
			else
			{
				fpout.write(reinterpret_cast<const char*> (&bufferSize), sizeof(UINT));
				fpout.write(pBuffer, bufferSize);
			}
			free(compressedBuffer);
		}
		else
		{
			fpout.write(reinterpret_cast<const char*> (&bufferSize), sizeof(UINT));
			fpout.write(pBuffer, bufferSize);
		}

		// Close file
		fpout.close ();
		return true;
	}

	bool readFrameImageHeader (const std::string& fileName, INT64* timeStamp, UINT* pWidth, UINT* pHeight)
	{
		// Open file in binary appendable
		std::ifstream fpin (fileName.c_str(), std::ios::in | std::ios::binary);
		if( !fpin.is_open() )
		{
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		if (timeStamp)
			fpin.read(reinterpret_cast<char*> (timeStamp), sizeof(INT64));
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
			LOG4CPLUS_FATAL(NuiLogger::instance().consoleLogger(), ("could not open file." + fileName).c_str());
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		fpin.seekg(2 * sizeof(UINT) + sizeof(INT64), std::ios::beg);
		if (pBuffer && bufferSize > 0)
		{
			UINT storeSize = bufferSize;
			fpin.read(reinterpret_cast<char*> (&storeSize), sizeof(UINT));
			if (storeSize != bufferSize)
			{
				Bytef* compressedBuffer = (Bytef*)malloc(sizeof(Bytef) * storeSize);
				fpin.read(reinterpret_cast<char*> (compressedBuffer), storeSize);
				if (Z_OK != uncompress((Bytef*)pBuffer, (unsigned long *)&bufferSize, compressedBuffer, storeSize))
				{
					LOG4CPLUS_FATAL(NuiLogger::instance().consoleLogger(), "failed to uncompress buffer.");
				}
				free(compressedBuffer);
			}
			else
			{
				fpin.read(pBuffer, bufferSize);
			}
		}

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
			LOG4CPLUS_FATAL(NuiLogger::instance().consoleLogger(), ("could not open file." + fileName).c_str());
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
			//LOG4CPLUS_FATAL(NuiLogger::instance().consoleLogger(), "could not open file." + fileName);
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

	bool		writeTimeLog (const std::string& fileName, const std::map<std::string, NuiTimeStamp>& timeMap)
	{
		// Open file
		std::ofstream fpout (fileName.c_str(), std::ios::out | std::ios::app);
		if( !fpout.is_open() )
		{
			LOG4CPLUS_FATAL(NuiLogger::instance().consoleLogger(), ("could not open file." + fileName).c_str());
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		std::map<std::string, NuiTimeStamp>::const_iterator iter;
		for(iter = timeMap.begin();iter != timeMap.end(); ++iter)
		{
			
			fpout << iter->first << " timeStamp:" << iter->second.m_sumTime << std::endl;
			fpout << iter->first << " count:" << iter->second.m_count << std::endl;
			fpout << iter->first << " fps:" << iter->second.m_count / iter->second.m_sumTime << std::endl;
		}
		
		// Close file
		fpout.close ();
		return true;
	}

	bool		writeDayTime (const std::string& fileName)
	{
		// Open file
		std::ofstream fpout (fileName.c_str(), std::ios::out | std::ios::app);
		if( !fpout.is_open() )
		{
			LOG4CPLUS_FATAL(NuiLogger::instance().consoleLogger(), ("could not open file." + fileName).c_str());
			//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
			return false;
		}

		time_t timep;
		time (&timep);
		fpout << ctime(&timep) << std::endl;

		// Close file
		fpout.close ();
		return true;
	}
}