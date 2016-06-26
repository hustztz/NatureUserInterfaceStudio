#pragma once

#include "stdafx.h"
#include "OpenCLUtilities/NuiOpenCLUtil.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Vector3i Vector3i;

class NuiKinfuColorVolume
{
public:
	NuiKinfuColorVolume(const Vector3i& resolution, unsigned char max_weight);
	~NuiKinfuColorVolume();

	/** \brief Returns volume resolution */
	const Vector3i&	getResolution() const;

	/** \brief Returns tsdf volume container that point to data in GPU memroy */
	cl_mem data() const;

	/** \brief Resets tsdf volume data to uninitialized state */
	void reset();
protected:
	
private:
	/** \brief tsdf volume data container */
	cl_mem	m_colorVolumeCL;
	/** \brief tsdf volume resolution */
	Vector3i m_resolution;

	unsigned char m_max_weight;
};