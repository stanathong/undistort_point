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
With the unknowns to be the undistorted coordinate, we want to minimise the error between this computed distorted image coordinate (x") with the measured value, let's say x. That is we want `x-x"` to be minimal. We do this by performing a non-linear optimisation using Ceres Solver.<br><br>
## Implementation
In the code, we implement `class CPointError`, defined in PointError.h to compute the residuals (for x and y coordinates) between the computed distorted coordinate and measured coordinate.
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
<br>
Then in the main function, we use ceres solver to minimise the sum of the squared residuals.
<br>
In the example, we would like to compute the undistorted coordinates of the four image corners i.e. top-left, top-right, bottom-right and bottom-left. On a side note, we may consider using (0.5, 0.5) instead of (0,0) for the top-left corner and so on to refer to the point as in the centre of the pixel.<br>

```

typedef std::vector< cv::Point2d >		VEC_CV2DPOINT;
VEC_CV2DPOINT vec_image_points;
vec_image_points.push_back(cv::Point2d(0.0, 0.0));			// top-left corner
vec_image_points.push_back(cv::Point2d(objCamera.m_img_width-1, 0.0));	// top-right corner
vec_image_points.push_back(cv::Point2d(objCamera.m_img_width-1, objCamera.m_img_height-1));	// bottom-right corner
vec_image_points.push_back(cv::Point2d(0.0, objCamera.m_img_height-1));	// bottom-left corner

```

<br>
We compute the undistorted coordinates for each corner independently as they are not related. <br>

```

// Initialise the output undistorted point to be the same as the distort point
double undistorted_x = vec_image_points[i].x; 
double undistorted_y = vec_image_points[i].y;

ceres::Problem problem;
ceres::CostFunction * Point_cost_function = 
				new ceres::AutoDiffCostFunction<CPointError, 2, 1, 1>(
						new CPointError(vec_image_points[i].x, vec_image_points[i].y,
										objCamera.m_fx, objCamera.m_fy,
										objCamera.m_cx, objCamera.m_cy,
										objCamera.m_k1, objCamera.m_k2,
										objCamera.m_p1, objCamera.m_p2) );
problem.AddResidualBlock(Point_cost_function, NULL, &undistorted_x, &undistorted_y);

// Set the solver option
ceres::Solver::Options options;
options.minimizer_progress_to_stdout = true;
options.max_num_iterations = 50; // Set to 50 iterations

ceres::Solver::Summary summary;
ceres::Solve(options, &problem, &summary);

```

<br>
The optimisation is finished within a few iterations.
