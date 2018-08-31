# Undistort Points
Computing distorted image coordinates can be done simply by multiplying/adding distortion coefficient to the undistorted coordinates (refer to the previous repository [retained_undistorted_image](https://github.com/stanathong/retained_undistort_image)). However, the reverse process i.e. computing undistorted image coordinates from the distorted (original) image coordinates, given the camera's intrinsics, requires a non-linear optimisation.<br><br>
The __OpenCV__ library has a function `cv::undistortPoints` to compute the undistorted coordinates easily. In this repository, we do this the hard way (?) through the use of the [Ceres Solver](http://ceres-solver.org/) to compute the undistorted coordinates by non-linearly minising the error between the input distorted coordinates and the computed distorted coordinates.<br>
<br>
![preview](https://github.com/stanathong/undistort_point/blob/master/figure/preview.jpg)
<br>
The picture on the left is the original image with lens distortion, while the picture on the right is the undistorted image (with the removal of lens distortion). The red lines show examples of corresponding image points between the two. <br>
## Math
We know that, given the camera parameters and lens distortion coefficient, we can compute the distorted image coordinate (x") for an undistorted image coordinate (x').<br>
<br>
![preview](https://github.com/stanathong/undistort_point/blob/master/figure/equation.jpg)
<br><br>
With the unknowns to be the undistorted coordinate, we want to minimise the error between this computed distorted image coordinate (x") with the measured value, let's say x. That is we want `x-x"` to be minimal. We do this by performing a linear optimisation using Ceres Solver.<br><br>
## Implementation
In the code, we implement `class CPointError`, defined in PointError.h to compute the error between the computed distorted coordinate and measured coordinate.
<br>
```
class CPointError
{
  ...
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
  ...
};
```
