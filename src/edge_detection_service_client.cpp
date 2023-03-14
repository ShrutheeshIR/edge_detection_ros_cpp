#include <ros/ros.h>
#include <edge_detection/DetectEdges.h>
#include <opencv2/opencv.hpp>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "Client node");


    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<edge_detection::DetectEdges>("detect_edges");
    edge_detection::DetectEdges srv;

	cv::Mat img = cv::imread("/home/shrutheesh/Desktop/neura/catkin_ws/src/edge_detection_ros_cpp/data/Image_1.png");
	std::cout << img.size() << std::endl;

    // srv.request.a = atoll(argv[1]);
    // srv.request.b = atoll(argv[2]);
    // if (client.call(srv))
    // {
    //     ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    // }
    // else
    // {
    //     ROS_ERROR("Failed to call service add_two_ints");
    //     return 1;
    // }

}