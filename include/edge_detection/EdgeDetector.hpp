#pragma once
#include <vector>

#include <opencv2/opencv.hpp>
// #include <eigen3/Eigen/Dense>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>
// using namespace Eigen;


enum LinesMethod {
	Hough = 0,
	Contour = 1
};

typedef struct Params{
	bool show_debug_image;
	double resize_ratio;
	int canny_lower_thresh;
	int canny_upper_thresh;
	LinesMethod detect_lines_method;
	double minSquareArea;
	double maxSquareArea;

	Params()
		: show_debug_image(false)
		, resize_ratio(1.0)
		, canny_lower_thresh(26)
		, canny_upper_thresh(225)
		, detect_lines_method(Hough)
		, minSquareArea(1000)
		, maxSquareArea(8000)
		{}
} Params;

class EdgeDetector
{
	// Your class declaration goes here
	Params params;

	public:
		EdgeDetector(bool show_debug, LinesMethod line_detection_method);
		void findEdges(cv::Mat& src_img, std::vector<cv::Point2d>& edge_points);
		void convertGrayscale(cv::Mat& src_img, cv::Mat& dst_img);
		void display_edges(cv::Mat &src_img, std::string filename, std::vector<cv::Point2d>& edge_points, cv::Mat& dst_img);
};

