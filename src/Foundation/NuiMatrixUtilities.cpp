#include "NuiMatrixUtilities.h"

#include <Eigen/Geometry>

namespace NuiMatrixUtilities
{
	Vector3f rodrigues2(const Eigen::Matrix3f& matrix)
	{
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(matrix, Eigen::ComputeFullV | Eigen::ComputeFullU);    
		Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();

		double rx = R(2, 1) - R(1, 2);
		double ry = R(0, 2) - R(2, 0);
		double rz = R(1, 0) - R(0, 1);

		double s = sqrt((rx*rx + ry*ry + rz*rz)*0.25);
		double c = (R.trace() - 1) * 0.5;
		c = c > 1. ? 1. : c < -1. ? -1. : c;

		double theta = acos(c);

		if( s < 1e-5 )
		{
			double t;

			if( c > 0 )
				rx = ry = rz = 0;
			else
			{
				t = (R(0, 0) + 1)*0.5;
				rx = sqrt( std::max(t, 0.0) );
				t = (R(1, 1) + 1)*0.5;
				ry = sqrt( std::max(t, 0.0) ) * (R(0, 1) < 0 ? -1.0 : 1.0);
				t = (R(2, 2) + 1)*0.5;
				rz = sqrt( std::max(t, 0.0) ) * (R(0, 2) < 0 ? -1.0 : 1.0);

				if( fabs(rx) < fabs(ry) && fabs(rx) < fabs(rz) && (R(1, 2) > 0) != (ry*rz > 0) )
					rz = -rz;
				theta /= sqrt(rx*rx + ry*ry + rz*rz);
				rx *= theta;
				ry *= theta;
				rz *= theta;
			}
		}
		else
		{
			double vth = 1/(2*s);
			vth *= theta;
			rx *= vth; ry *= vth; rz *= vth;
		}
		return Eigen::Vector3d(rx, ry, rz).cast<float>();
	}
}