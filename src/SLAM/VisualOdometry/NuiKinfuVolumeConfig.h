#pragma once

#include <Eigen/Core>

typedef Eigen::Vector3f Vector3f;
typedef Eigen::Vector3i Vector3i;

struct NuiKinfuVolumeConfig
{
	/** \brief tsdf volume size in meters */
	Vector3f	dimensions;

	/** \brief tsdf volume resolution */
	Vector3i	resolution;

	 /** \brief Sets Tsdf truncation distance. Must be greater than 2 * volume_voxel_size
    * \param[in] distance TSDF truncation distance 
    */
	float		tranc_dist; //meters
	bool		bHas_color_volume;
	int			max_color_weight;

	bool		bIsDynamic;
	int			voxel_shift;
	Vector3f	translateBasis;

	NuiKinfuVolumeConfig();

	bool	load(const std::string& fileName);
	bool	log(const std::string& fileName) const;
};