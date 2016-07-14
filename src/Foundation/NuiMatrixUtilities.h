#pragma once

#include <Eigen/Core>

typedef Eigen::Vector3f Vector3f;

namespace NuiMatrixUtilities
{
	Vector3f rodrigues2(const Eigen::Matrix3f& matrix);
}