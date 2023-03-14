#include "edge_detection/edge_detection_node.h"

EdgeDetectionNode :: EdgeDetectionNode()
{
    det = new EdgeDetector(false, Hough);
    sub = nh_.subscribe("image_raw", 1000, &EdgeDetectionNode::edge_detection_image_callback, this);
    service = n.advertiseService("add_two_ints", add);
}

void EdgeDetectionNode :: detect_edges()
{
    ;
}

void EdgeDetectionNode :: edge_detection_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    ;
}


// void EdgeDetectionNode::detect_edges()
// {
//     ;
// }

void EdgeDetectionNode::edge_detection(edge_detection::DetectEdges::Request &request, edge_detection::DetectEdges::Response &response)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(request.img, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_img = cv::Mat(cv_ptr -> image);
    
    std::vector<cv::Point2d> edge_points;
    det->findEdges(cv_img, edge_points);

    det->display_edges(cv_img, edge_points);

}


int main()
{
    return 0;
}