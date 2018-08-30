#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>	// cv::remap
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>

#include "PointError.h"
#include "Intrinsics.h"

typedef std::vector< cv::Point2d >		VEC_CV2DPOINT;

int main(int argc, char *argv[])
{
	// 1. Read the camera parameters from the passed-in yaml file or the default file location
	std::string camera_filepath = (argc == 1) ? "camera.yaml" : argv[1];
	CIntrinsics objCamera(camera_filepath);

	// 2. (Manually) Define the (distorted) image points to compute undistorted coordinates
	//    Modify this vector to include other image points (x,y) to be computed.
	VEC_CV2DPOINT vec_image_points;
	vec_image_points.push_back(cv::Point2d(0.0, 0.0));						// top-left corner
	vec_image_points.push_back(cv::Point2d(objCamera.m_img_width-1, 0.0));	// top-right corner
	vec_image_points.push_back(cv::Point2d(objCamera.m_img_width-1, objCamera.m_img_height-1));	// bottom-right corner
	vec_image_points.push_back(cv::Point2d(0.0, objCamera.m_img_height-1));	// bottom-left corner

	// 3. Compute the undistorted coordinates for each image points in the vector
	for (int i = 0; i < vec_image_points.size(); i++)
	{
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
		// Uncomment to see the full report
		//std::cout << summary.FullReport() << std::endl;

		// Print the result if the problem is convergent
		if (ceres::TerminationType::CONVERGENCE == summary.termination_type) 
		{
			std::cout	<< "Distorted: (" << vec_image_points[i].x << ", " << vec_image_points[i].y << ") vs Undistorted: "
						<< undistorted_x << ", " << undistorted_y << ")" << std::endl;
		}
		else
		{
			std::cout	<< "Failed to compute undistorted coordinates for point (" 
						<< vec_image_points[i].x << ", " << vec_image_points[i].y << ")!" << std::endl;
		}
	}

	return 0;
}