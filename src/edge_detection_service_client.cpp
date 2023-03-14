#include <ros/ros.h>
#include <edge_detection/DetectEdges.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <filesystem>
#include <string>
// #include <experimental/filesystem>

namespace fs = std::filesystem;

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ClientNode");


    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<edge_detection::DetectEdges>("detect_edges");
    edge_detection::DetectEdges srv;


    std::string path = "/neura/src/edge_detection_ros_cpp/data/";
    for (const auto & entry : fs::directory_iterator(path))
    {

        cv::Mat img = cv::imread(entry.path());
        std::cout << img.size() << std::endl;


        cv_bridge::CvImage img_bridge;
        sensor_msgs::Image img_msg; // >> message to be sent

        std_msgs::Header header; // empty header
        header.stamp = ros::Time::now(); // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
        img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image

        srv.request.img = img_msg;
        if (client.call(srv))
        {
            ROS_INFO("Called!");

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.edge_output.img, sensor_msgs::image_encodings::RGB8);
            cv::Mat cv_img = cv::Mat(cv_ptr -> image);
            std::string filename = entry.path().filename();
            std::string dirname = "/neura/src/edge_detection_ros_cpp/output/";
            cv::imwrite(dirname + filename, cv_img);

        }
        // break;
    }

}