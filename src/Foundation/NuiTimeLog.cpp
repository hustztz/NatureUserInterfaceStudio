#include "NuiTimeLog.h"

#include "NuiFileIOUtilities.h"

#include <iostream>

#ifdef _WIN32
#include <windows.h>
#endif

NuiTimeLog& NuiTimeLog::instance() {
	static NuiTimeLog instance;
	return instance;
}

NuiTimeLog::NuiTimeLog()
	: m_bEnable(true)
{
	LARGE_INTEGER qpf = {0};
	if (QueryPerformanceFrequency(&qpf))
	{
		m_timeFreq = double(qpf.QuadPart);
	}
	else
	{
		m_timeFreq = 0.f;
	}

	reset();
}

NuiTimeLog::~NuiTimeLog()
{
}

void	NuiTimeLog::reset()
{
	LARGE_INTEGER tTimeStamp = {0};
	QueryPerformanceCounter(&tTimeStamp);

	TimeStampMap::iterator iter;
	for(iter = m_timeMap.begin();iter != m_timeMap.end(); ++iter)
	{
		NuiTimeStamp timeStamp;
		timeStamp.m_count = 0;
		timeStamp.m_currentTime = tTimeStamp.QuadPart;
		timeStamp.m_sumTime = 0;
		iter->second = timeStamp;
	}
}

void	NuiTimeLog::tick(const std::string& name)
{
	if(!m_bEnable)
		return;

	LARGE_INTEGER tProcessStartTimeStamp = {0};
	QueryPerformanceCounter(&tProcessStartTimeStamp);

	TimeStampMap::iterator iter = m_timeMap.find(name);
	if(iter == m_timeMap.end())
	{
		NuiTimeStamp defaultTimeStamp;
		defaultTimeStamp.m_count = 0;
		defaultTimeStamp.m_currentTime = tProcessStartTimeStamp.QuadPart;
		defaultTimeStamp.m_sumTime = 0;
		m_timeMap.insert(std::make_pair(name, defaultTimeStamp));
	}
	else
	{
		iter->second.m_currentTime = tProcessStartTimeStamp.QuadPart;
	}
}

void	NuiTimeLog::tock(const std::string& name)
{
	if(!m_bEnable)
		return;

	TimeStampMap::iterator iter = m_timeMap.find(name);
	if(iter == m_timeMap.end())
		return;

	LARGE_INTEGER tProcessEndTimeStamp = {0};
	QueryPerformanceCounter(&tProcessEndTimeStamp);

	iter->second.m_sumTime += tProcessEndTimeStamp.QuadPart - iter->second.m_currentTime;
	iter->second.m_count ++;
	iter->second.m_currentTime = tProcessEndTimeStamp.QuadPart;
}

double	NuiTimeLog::avgFPS(const std::string& name) const
{
	TimeStampMap::const_iterator iter = m_timeMap.find(name);
	if(iter == m_timeMap.end())
		return 0.0;

	return m_timeFreq * iter->second.m_count / (double(iter->second.m_sumTime) + 0.05);
}

void	NuiTimeLog::print() const
{
	TimeStampMap::const_iterator iter;
	for(iter = m_timeMap.begin();iter != m_timeMap.end(); ++iter)
	{
		std::cout << iter->first << " fps:" << m_timeFreq * iter->second.m_count / (double(iter->second.m_sumTime) + 0.05)  << std::endl;
		std::cout << iter->first << " count:" << iter->second.m_count  << std::endl;
	}
}

void	NuiTimeLog::toFile(const std::string& fileName) const
{
	if ( !NuiFileIOUtilities::writeTime(fileName, m_timeMap, m_timeFreq) )
	{
		std::cout << "Failed to log time to file." << std::endl;
	}
}