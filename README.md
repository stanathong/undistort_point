# Undistort Points
Computing distorted image coordinates can be done simply by multiplying/adding distortion coefficient to the undistorted coordinates (refer to the previous repository [retained_undistorted_image](https://github.com/stanathong/retained_undistort_image)). However, the reverse process i.e. computing undistorted image coordinates from the distorted (original) image coordinates, given the camera's intrinsics, requires a non-linear optimisation.<br><br>
The __OpenCV__ library has a function `cv::undistortPoints` to compute the undistorted coordinates easily. In this repository, we do this the hard way (?) through the use of the [Ceres Solver](http://ceres-solver.org/) to compute the undistorted coordinates by non-linearly minising the error between the input distorted coordinates and the computed distorted coordinates.<br>
<br>
![preview](https://github.com/stanathong/undistort_point/blob/master/figure/preview.jpg)
<br>
The picture on the left is the original image with lens distortion, while the picture on the right is the undistorted image (with the removal of lens distortion). The red lines show examples of corresponding image points between the two. <br>
## Math
We know that, given the camera parameters and lens distortion coefficient, we can compute the distorted image coordinate (x") for an undistorted image coordinate (x').<br>
![preview](https://github.com/stanathong/undistort_point/blob/master/figure/equation.jpg)
<br>
With the unknowns to be the undistorted coordinate, we want to minimise the error between this computed distorted image coordinate (x") with the measured value, let's say x. That is we want `x-x"` to be minimal. We do this by performing a linear optimisation using Ceres Solver.<br>
## Implementation
In the code, we implement `class CPointError` to compute the error between the computed distorted coordinate and measured coordinate.
