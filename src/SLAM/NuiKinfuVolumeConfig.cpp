#include "NuiKinfuVolumeConfig.h"

#include <fstream>

NuiKinfuVolumeConfig::NuiKinfuVolumeConfig()
{
	dimensions = Vector3f::Constant(3.0f);
	resolution = Vector3i::Constant(256);
	tranc_dist = 0.03f;
	bHas_color_volume = true;
	max_color_weight = 128;
	bIsDynamic = false;
	voxel_shift = 15;
	translateBasis = Vector3f::Zero();
}

bool	NuiKinfuVolumeConfig::load(const std::string& fileName)
{
	std::ifstream fpin (fileName.c_str(), std::ios::in);
	if( !fpin.is_open() )
	{
		//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
		return false;
	}

	std::string line;
	while( getline(fpin,line) )
	{
		size_t pos = line.find('=');
		if(pos == std::string::npos)
			continue;

		std::string tmpKey = line.substr(0, pos);
		if("VolumeSizeX" == tmpKey)
		{
			dimensions[0] = (float)atof(line.substr(pos+1).c_str());
		}
		else if("VolumeSizeY" == tmpKey)
		{
			dimensions[1] = (float)atof(line.substr(pos+1).c_str());
		}
		else if("VolumeSizeZ" == tmpKey)
		{
			dimensions[2] = (float)atof(line.substr(pos+1).c_str());
		}
		else if("VolumeResolutionX" == tmpKey)
		{
			resolution[0] = atoi(line.substr(pos+1).c_str());
		}
		else if("VolumeResolutionY" == tmpKey)
		{
			resolution[1] = atoi(line.substr(pos+1).c_str());
		}
		else if("VolumeResolutionZ" == tmpKey)
		{
			resolution[2] = atoi(line.substr(pos+1).c_str());
		}
		else if("VolumeTrancDist" == tmpKey) 
		{
			tranc_dist = (float)atof(line.substr(pos+1).c_str());
		}
		else if("VolumeHadColor" == tmpKey) 
		{
			bHas_color_volume = atoi(line.substr(pos+1).c_str()) ? true : false;
		}
		else if("MaxColorWeight" == tmpKey) 
		{
			max_color_weight = atoi(line.substr(pos+1).c_str());
		}
		else if("IsDynamicVolume" == tmpKey)
		{
			bIsDynamic = atoi(line.substr(pos+1).c_str()) ? true : false;
		}
		else if("DynamicVoxelShift" == tmpKey) 
		{
			voxel_shift = atoi(line.substr(pos+1).c_str());
		}
		else if("VolumeBasisX" == tmpKey) 
		{
			translateBasis[0] = (float)atof(line.substr(pos+1).c_str());
		}
		else if("VolumeBasisY" == tmpKey) 
		{
			translateBasis[1] = (float)atof(line.substr(pos+1).c_str());
		}
		else if("VolumeBasisZ" == tmpKey) 
		{
			translateBasis[2] = (float)atof(line.substr(pos+1).c_str());
		}
	}

	// Close file
	fpin.close ();
	return true;
}

bool	NuiKinfuVolumeConfig::log(const std::string& fileName)
{
	// Open file
	std::ofstream fpout (fileName.c_str(), std::ios::out | std::ios::app);
	if( !fpout.is_open() )
	{
		//throw std::ios::failure(__FUNCTION__ + std::string(": could not open file ") + fileName);
		return false;
	}

	fpout << "VolumeSizeX=" << dimensions[0] << std::endl;
	fpout << "VolumeSizeY=" << dimensions[1] << std::endl;
	fpout << "VolumeSizeZ=" << dimensions[2] << std::endl;
	fpout << "VolumeResolutionX=" << resolution[0] << std::endl;
	fpout << "VolumeResolutionY=" << resolution[1] << std::endl;
	fpout << "VolumeResolutionZ=" << resolution[2] << std::endl;
	fpout << "VolumeTrancDist=" << tranc_dist << std::endl;
	fpout << "VolumeHadColor=" << bHas_color_volume << std::endl;
	fpout << "MaxColorWeight=" << max_color_weight << std::endl;
	fpout << "IsDynamicVolume=" << bIsDynamic << std::endl;
	fpout << "DynamicVoxelShift=" << voxel_shift << std::endl;
	fpout << "VolumeBasisX=" << translateBasis[0] << std::endl;
	fpout << "VolumeBasisY=" << translateBasis[1] << std::endl;
	fpout << "VolumeBasisZ=" << translateBasis[2] << std::endl;

	// Close file
	fpout.close ();
	return true;
}