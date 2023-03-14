#include <opencv2/opencv.hpp>

cv::Mat read_image(std::string filename)
{
    cv::Mat img = cv::imread(filename);
    return img;

}