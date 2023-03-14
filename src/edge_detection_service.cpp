#include "edge_detection/edge_detection_service.h"

EdgeDetectionService :: EdgeDetectionService()
{
    det = new EdgeDetector(false, Hough);
    service = nh_.advertiseService("detect_edges", &EdgeDetectionService::edge_detection, this);
}

bool EdgeDetectionService::edge_detection(edge_detection::DetectEdges::Request &request, edge_detection::DetectEdges::Response &response)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(request.img, sensor_msgs::image_encodings::RGB8);
    cv::Mat cv_img = cv::Mat(cv_ptr -> image);
    
    std::vector<cv::Point2d> edge_points;
    det->findEdges(cv_img, edge_points);

    cv::Mat dst_img;
    // det->display_edges(cv_img, "/neura/src/edge_detection_ros_cpp/output/edges.png", edge_points, dst_img);


    geometry_msgs::Point pt;
    std::vector<geometry_msgs::Point> points;
    for(cv::Point2d edge_point: edge_points)
    {
        pt.x = edge_point.x;
        pt.y = edge_point.y;
        pt.z = 0;
        points.push_back(pt);
    }

    cv_bridge::CvImage img_bridge;
    sensor_msgs::Image img_msg; // >> message to be sent

    std_msgs::Header header = cv_ptr->header; // empty header
    header.stamp = ros::Time::now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, dst_img);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image


    response.edge_output.img = img_msg;
    response.edge_output.edges_2d = points;

    return true;


    // saliency_img_pub.publish(out_msg.toImageMsg());    

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RosInit");
    EdgeDetectionService node = EdgeDetectionService();
//   ros::ServiceServer service = n.advertiseService("detect_edges", add);
    ROS_INFO("Server Setup!.");
    ros::spin();
}