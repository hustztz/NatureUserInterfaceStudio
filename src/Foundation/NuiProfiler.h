#pragma once

#include <map>

namespace boost
{
	namespace timer{ class cpu_timer; }
}
class NuiProfiler
{
public:
	NuiProfiler& getSingleton();
	void enable(){enabled = true;}
	void disable(){enabled = false;}
	void start( const std::string& name );
	void stop( const std::string& name );
private:
	static NuiProfiler& profiler;
	std::map<std::string, boost::timer::cpu_timer*> start_time;
	bool enabled;

	NuiProfiler();
};