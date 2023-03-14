#include "edge_detection/edge_detection_service.h"

EdgeDetectionService :: EdgeDetectionService()
{
    det = new EdgeDetector(false, Hough);
    service = nh_.advertiseService("detect_edges", &EdgeDetectionService::edge_detection, this);
}

bool EdgeDetectionService::edge_detection(edge_detection::DetectEdges::Request &request, edge_detection::DetectEdges::Response &response)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(request.img, sensor_msgs::image_encodings::BGR8);
    cv::Mat cv_img = cv::Mat(cv_ptr -> image);
    
    std::vector<cv::Point2d> edge_points;
    det->findEdges(cv_img, edge_points);

    cv::Mat dst_img;
    det->display_edges(cv_img, edge_points, dst_img);


    cv_bridge::CvImage out_msg;
    out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    out_msg.image    = dst_img; // Your cv::Mat

    geometry_msgs::Point pt;
    std::vector<geometry_msgs::Point> points;
    for(cv::Point2d edge_point: edge_points)
    {
        pt.x = edge_point.x;
        pt.y = edge_point.y;
        pt.z = 0;
        points.push_back(pt);
    }

    // response.edge_output.img = out_msg.toImageMsg();
    response.edge_output.edges_2d = points;

    return true;


    // saliency_img_pub.publish(out_msg.toImageMsg());    

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "Ros init");
    EdgeDetectionService node = EdgeDetectionService();
//   ros::ServiceServer service = n.advertiseService("detect_edges", add);
    ROS_INFO("Server Setup!.");
    ros::spin();
}