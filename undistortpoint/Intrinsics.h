#ifndef _INTRINSICS_H_
#define _INTRINSICS_H_

#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>

class CIntrinsics
{
public: 
	CIntrinsics(const std::string &camera_filepath)
	{
		// Read the camera file
		cv::FileStorage fs(camera_filepath, cv::FileStorage::READ);
		if (!fs.isOpened())
		{
			std::cout << "Failed to read file: " << camera_filepath << ", use the default parameters." << std::endl;

			// Assign the default parameters
			Init();
		}
		else
		{
			m_img_width = static_cast<int>(fs["image_width"]);
			m_img_height = static_cast<int>(fs["image_height"]);
			m_fy = static_cast<double>(fs["fy"]);
			m_fx = static_cast<double>(fs["fx"]);
			m_fy = static_cast<double>(fs["fy"]);
			m_cx = static_cast<double>(fs["cx"]);
			m_cy = static_cast<double>(fs["cy"]);
			m_k1 = static_cast<double>(fs["k1"]);
			m_k2 = static_cast<double>(fs["k2"]);
			m_p1 = static_cast<double>(fs["p1"]);
			m_p2 = static_cast<double>(fs["p2"]);
		}
	}

	// Assign the default parameters
	void Init()
	{
		// Assign the width and height of the image
		m_img_width = 1920;
		m_img_height = 1080;

		// Initialise intrinsics parameters
		m_fx = 1738.06409;
		m_fy = 1736.96128;
		m_cx = 965.22200;
		m_cy = 666.61850;
	
		// Initialise distortion parameters
		m_k1 = -0.34592;
		m_k2 = 0.16969;
		m_p1 = -0.00279;
		m_p2 = 0.00235;
	}

public:
	int				m_img_width;		// Image width
	int				m_img_height;		// Image height
	double			m_k1;				// Radial distortion factor 1
	double			m_k2;				// Radial distortion factor 2
	double			m_p1;				// Tangential distortion factor 1
	double			m_p2;				// Tangential distortion factor 2
	double			m_fx;				// Focal length x
	double			m_fy;				// Focal length y
	double			m_cx;				// Optical center x
	double			m_cy;				// Optical center y
};

#endif
