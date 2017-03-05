#pragma once

#include <Eigen/Core>


typedef Eigen::Vector3f Vector3f;
#define NAN_FLOAT -std::numeric_limits<float>::max()

static bool _IsNan(Vector3f data)
{
	return ((NAN_FLOAT == data[0]) || (NAN_FLOAT == data[1]) || (NAN_FLOAT == data[2]) || 
	_isnan(data[0]) || _isnan(data[1]) || _isnan(data[2])); 
}
