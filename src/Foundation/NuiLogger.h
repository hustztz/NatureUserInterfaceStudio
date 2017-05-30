#pragma once

#include <log4cplus/logger.h>
#include <log4cplus/loggingmacros.h>
#include <log4cplus/fileappender.h>

// LOG4CPLUS_DEBUG
// LOG4CPLUS_INFO
// LOG4CPLUS_WARN
// LOG4CPLUS_ERROR
// LOG4CPLUS_FATAL
class NuiLogger
{
public:
	
	static NuiLogger& instance();

	void	setQuietMode();

	const log4cplus::Logger& consoleLogger() const { return m_consolelogger; }
	const log4cplus::Logger& fileLogger() const { return m_filelogger; }

private:
	log4cplus::Logger				m_consolelogger;
	log4cplus::SharedAppenderPtr	m_consoleAppender;
	log4cplus::Logger				m_filelogger;
	log4cplus::SharedFileAppenderPtr	m_fileAppender;

private:
	NuiLogger();
	~NuiLogger();
};
