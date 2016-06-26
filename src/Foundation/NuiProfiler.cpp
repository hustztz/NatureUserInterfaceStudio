#include <boost/timer/timer.hpp>
#include "NuiProfiler.h"

NuiProfiler& NuiProfiler::profiler = NuiProfiler();

using boost::timer::cpu_timer;

NuiProfiler::NuiProfiler()
	: enabled(false)
{
	start_time = std::map<std::string, cpu_timer*>();
}

NuiProfiler& NuiProfiler::getSingleton()
{
	return profiler;
}

void NuiProfiler::start(const std::string& name)
{
	if(enabled)
	{
		cpu_timer* timer = new cpu_timer();
		timer->start();
		start_time[name] = timer;
	}
}

void NuiProfiler::stop(const std::string& name)
{
	if(enabled)
	{
		auto pos = start_time.find(name);
		if( pos == start_time.end())
			return;

		cpu_timer* timer = start_time[name];
		start_time[name]->stop();
		std::cout << name << " elapsed time : " << boost::timer::format(timer->elapsed(), 5, "%t") << std::endl;

		delete timer;
		start_time.erase(pos);
	}
}
