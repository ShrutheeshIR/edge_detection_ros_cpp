#include "EdgeDetector.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "edge_detection/DetectEdges.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


class EdgeDetectionNode
{
    EdgeDetector *det;
    ros::NodeHandle nh_;
    public:
        EdgeDetectionNode();
        void detect_edges();
        void edge_detection(edge_detection::DetectEdges::Request &request, edge_detection::DetectEdges::Response &response);
        ros::Subscriber sub;

        void edge_detection_image_callback(const sensor_msgs::ImageConstPtr& msg);

        ros::ServiceServer service;

};

