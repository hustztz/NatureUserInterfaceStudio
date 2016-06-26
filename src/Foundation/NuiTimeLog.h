#pragma once

#include "stdafx.h"

#include "NuiTimeStamp.h"

#include <map>
#include <string>

class NuiTimeLog
{
public:
	
	static NuiTimeLog& instance();

	void	reset();

	void	tick(const std::string& name);
	void	tock(const std::string& name);
	double	avgFPS(const std::string& name) const;

	void	print() const;
	void	toFile(const std::string& fileName) const;

private:
	bool				m_bEnable;
	double				m_timeFreq;

	typedef std::map<std::string, NuiTimeStamp> TimeStampMap;
	TimeStampMap		m_timeMap;

private:
	NuiTimeLog();
	~NuiTimeLog();
};
