#ifndef _POINTERROR_H_
#define _POINTERROR_H_

#pragma once

#include <ceres/ceres.h>

class CPointError
{
public: 
	CPointError(double _distorted_x, double _distorted_y,
				double _fx, double _fy, double _cx, double _cy,
				double _k1, double _k2, double _p1, double _p2)
	{
		distorted_x = _distorted_x;
		distorted_y = _distorted_y;
		fx = _fx;
		fy = _fy; 
		cx = _cx; 
		cy = _cy;
		k1 = _k1;
		k2 = _k2; 
		p1 = _p1; 
		p2 = _p2;
	}

	template <typename T>
	bool operator() (const T* const undistorted_x, const T* const undistorted_y, T* residuals) const
	{
		T x = (undistorted_x[0] - T(cx)) / T(fx);
		T y = (undistorted_y[0] - T(cy)) / T(fy);
		T r2 = x*x + y*y;
		T kcoef = T(1.0) + T(k1)*r2 + T(k2)*r2*r2;
		T x_ = x*kcoef + T(2.0)*T(p1)*x*y + T(p2)*(r2 + T(2.0)*x*x);
		T y_ = y*kcoef + T(p1)*(r2 + T(2.0)*y*y) + T(2.0)*T(p2)*x*y;
		T computed_distorted_x = x_*T(fx) + T(cx);
		T computed_distorted_y = y_*T(fy) + T(cy);

		// Compute the residuals between the measured distorted and computed distorted coords.
		residuals[0] = T(distorted_x) - computed_distorted_x;
		residuals[1] = T(distorted_y) - computed_distorted_y;

		return true;
	}

private:
	double distorted_x, distorted_y;
	double fx, fy, cx, cy;
	double k1, k2, p1, p2;
};

#endif
