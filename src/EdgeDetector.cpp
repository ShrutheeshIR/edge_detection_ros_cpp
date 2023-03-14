#include <edge_detection/EdgeDetector.hpp>

// Your class methods definition goes here

// EdgeDetector::get_gradients(const cv::Mat& img, cv::Mat& du, cv::Mat& dv, cv::Mat& angle)
// {

// }

EdgeDetector::EdgeDetector(bool show_debug = false, LinesMethod line_detection_method = Hough)
{
	params.show_debug_image = show_debug;
	params.detect_lines_method = line_detection_method;
}

void EdgeDetector::findEdges(cv::Mat& src_img, std::vector<cv::Point2d>& edge_points)
{
	cv::Mat img;
	cv::Size img_size = src_img.size();
	// cv::resize(src_img, img, cv::Size(img_size.height * params.resize_ratio, img_size.width * params.resize_ratio));
	img = src_img.clone();
	convertGrayscale(img, img);

	cv::Mat kernel = cv::getStructuringElement(
		cv::MORPH_RECT, cv::Size(2, 2)
	);
	cv::dilate(img, img, kernel);

	cv::Mat thresh;
	cv::inRange(img, cv::Scalar(0), cv::Scalar(130), thresh);
	cv::Mat edges_image;
	cv::Canny(thresh, edges_image, 10, 250);
	if(params.show_debug_image)
	{
		cv::imwrite("/home/olorin/Desktop/Neura/edge_detection_ros_challenge/edge_detection_ros/src/edge_detection/output/edges_image.png", edges_image);
	}

	if(params.detect_lines_method == Hough)
	{
		std::vector<cv::Vec2f> lines;
		cv::HoughLines(edges_image, lines, 1, CV_PI/180, 150, 0, 0);

		

		for( size_t i = 0; i < lines.size(); i++ )
		{
			float rho = lines[i][0], theta = lines[i][1];
			cv::Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			cv::LineIterator it(img, pt1, pt2);
			for(int i = 0; i < it.count; i++, ++it)
				edge_points.push_back(it.pos());
		}

	}
	else if (params.detect_lines_method == Contour)
	{
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		findContours(thresh, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
		std::vector<cv::Point2d> valid_edge_points;
		for(std::vector<cv::Point> contour: contours)
		{
			double area = cv::contourArea(contour);
			if(area > params.minSquareArea && area < params.maxSquareArea)
				valid_edge_points.insert(valid_edge_points.end(), contour.begin(), contour.end());
		}
		edge_points = valid_edge_points;
		
	}
	cv::Mat dst_img;
	if(params.show_debug_image)
		display_edges(src_img, edge_points, dst_img);
	

}

void EdgeDetector::display_edges(cv::Mat& src_img, std::vector<cv::Point2d>& edge_points, cv::Mat& dst_img)
{
	dst_img = src_img.clone();
	for(cv::Point2d edge_point: edge_points)
	{
		circle(dst_img, edge_point,1,CV_RGB(0,255,0),1);

	}	
	// cv::imwrite(filename, dst_img);

}

void EdgeDetector::convertGrayscale(cv::Mat& src_img, cv::Mat& dst_img)
{
	cv::cvtColor(src_img, dst_img, cv::COLOR_BGR2GRAY);

}

// int main()
// {
// 	// edge_detection::EdgeDetector detector = EdgeDetector(true, Hough);
// 	EdgeDetector detector;
// 	detector = EdgeDetector(true, Hough);
// 	cv::Mat img = cv::imread("/home/olorin/Desktop/Neura/edge_detection_ros_challenge/edge_detection_ros/src/edge_detection/data/Image_3.png");
// 	std::cout << img.size() << std::endl;

// 	std::vector<cv::Point2d> edges;

// 	detector.findEdges(img, edges);
//     // Create the executable for testing the code here

// 	return 0;
// }
