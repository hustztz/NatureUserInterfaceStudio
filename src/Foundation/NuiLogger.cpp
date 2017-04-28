#include "NuiLogger.h"

#include <log4cplus/initializer.h>
#include <log4cplus/configurator.h>  
#include <log4cplus/consoleappender.h>
#include <log4cplus/fileappender.h>
#include <log4cplus/helpers/loglog.h>

#define LOG4CPLUS_CONF_FILE "./log4cplus.properties"

using namespace log4cplus;
using namespace log4cplus::helpers;

NuiLogger& NuiLogger::instance() {
	static NuiLogger instance;
	return instance;
}

NuiLogger::NuiLogger()
	: m_consoleAppender(new ConsoleAppender())
	, m_fileAppender(new RollingFileAppender(LOG4CPLUS_TEXT("Test.log"), 20 * 1024, 5,
		false, true))
{
	log4cplus::Initializer initializer;

	//PropertyConfigurator::doConfigure(LOG4CPLUS_TEXT(LOG4CPLUS_CONF_FILE));
#ifdef _DEBUG
	LogLog::getLogLog()->setInternalDebugging(true);
#endif // _DEBUG
	
	m_fileAppender->setName(LOG4CPLUS_TEXT("fileAppender"));
	log4cplus::tstring pattern = LOG4CPLUS_TEXT("%d{%m/%d/%y %H:%M:%S,%Q} [%t] %-5p %c{2} %%%x%% - %X{key} - %m [%l]%n");
	m_fileAppender->setLayout(std::unique_ptr<Layout>(new PatternLayout(pattern)));

	//m_fileAppender->addFilter(spi::FilterPtr(new FunctionFilter(filterFunction)));

	m_consoleAppender->setLayout(std::unique_ptr<Layout>(new TTCCLayout()));

	Logger root = Logger::getRoot();
	LOG4CPLUS_DEBUG(root,
		"This is"
		<< " a reall"
		<< "y long message." << std::endl
		<< "Just testing it out" << std::endl
		<< "What do you think?");

	m_consolelogger = Logger::getInstance(LOG4CPLUS_TEXT("consoleLogger"));
	m_consolelogger.addAppender(m_consoleAppender);

	m_filelogger = Logger::getInstance(LOG4CPLUS_TEXT("fileLogger"));
	m_filelogger.addAppender(m_fileAppender);
	m_filelogger.setLogLevel(INFO_LOG_LEVEL);
}

NuiLogger::~NuiLogger()
{
	m_consolelogger.shutdown();
	m_filelogger.shutdown();
}

void	NuiLogger::setQuietMode()
{
	LogLog::getLogLog()->setQuietMode(true);
}

