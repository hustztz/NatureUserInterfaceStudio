#include "NuiICPConfig.h"

#include <fstream>

NuiICPConfig::NuiICPConfig()
{
	filter_radius = 3; //static_cast<int>(sigma_space * 1.5);
	sigma_space2_inv_half = 0.08f; // 0.5f / (sigma_space * sigma_space) //const float sigma_space = 2.5;     // in pixels
	sigma_depth2_inv_half = 50.0f; // 0.5f / (sigma_color * sigma_color) // const float sigma_color = 0.3;     //in meter

	depth_threshold = 0.3f; //meters
	dist_threshold = 0.1f; //meters
	normal_threshold = sin (20.f * 3.14159254f / 180.f); //0.8f;
	track_threshold = 0.15f;
	iterations.push_back( 5 );
	iterations.push_back( 5 );
	iterations.push_back( 5 );
}

bool	NuiICPConfig::load(const std::string& fileName)
{
	std::ifstream fpin (fileName.c_str(), std::ios::in);
	if( !fpin.is_open() )
	{
		//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
		return false;
	}

	iterations.clear();
	std::string line;
	while( getline(fpin,line) )
	{
		size_t pos = line.find('=');
		if(pos == std::string::npos)
			continue;

		std::string tmpKey = line.substr(0, pos);
		if("icp_filter_radius" == tmpKey)
		{
			filter_radius = (UINT)atoi(line.substr(pos+1).c_str());
		}
		else if("icp_sigma_space2_inv_half" == tmpKey) 
		{
			sigma_space2_inv_half = (float)atof(line.substr(pos+1).c_str());
		}
		else if("icp_sigma_depth2_inv_half" == tmpKey) 
		{
			sigma_depth2_inv_half = (float)atof(line.substr(pos+1).c_str());
		}
		else if("icp_depth_threshold" == tmpKey) 
		{
			depth_threshold = (float)atof(line.substr(pos+1).c_str());
		}
		else if("icp_dist_threshold" == tmpKey)
		{
			dist_threshold = (float)atof(line.substr(pos+1).c_str());
		}
		else if("icp_normal_threshold" == tmpKey) 
		{
			normal_threshold = (float)atof(line.substr(pos+1).c_str());
		}
		else if("icp_track_threshold" == tmpKey) 
		{
			track_threshold = (float)atof(line.substr(pos+1).c_str());
		}
		else if("icp_iteration" == tmpKey) 
		{
			UINT iter = (UINT)atoi(line.substr(pos+1).c_str());
			if(iter > 0)
				iterations.push_back(iter);
		}
	}

	// Close file
	fpin.close ();
	return true;
}

bool	NuiICPConfig::log(const std::string& fileName)
{
	// Open file
	std::ofstream fpout (fileName.c_str(), std::ios::out | std::ios::app);
	if( !fpout.is_open() )
	{
		//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
		return false;
	}

	fpout << "icp_filter_radius=" << filter_radius << std::endl;
	fpout << "icp_sigma_space2_inv_half=" << sigma_space2_inv_half << std::endl;
	fpout << "icp_sigma_depth2_inv_half=" << sigma_depth2_inv_half << std::endl;
	fpout << "icp_depth_threshold=" << depth_threshold << std::endl;
	fpout << "icp_dist_threshold=" << dist_threshold << std::endl;
	fpout << "icp_normal_threshold=" << normal_threshold << std::endl;
	fpout << "icp_track_threshold=" << track_threshold << std::endl;
	for (UINT i = 0; i < iterations.size(); i ++)
	{
		fpout << "icp_iteration=" << iterations[i] << std::endl;
	}

	// Close file
	fpout.close ();
	return true;
}