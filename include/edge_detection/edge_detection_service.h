#include "EdgeDetector.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "edge_detection/DetectEdges.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


class EdgeDetectionService
{
    EdgeDetector *det;
    ros::NodeHandle nh_;
    public:
        EdgeDetectionService();
        bool edge_detection(edge_detection::DetectEdges::Request &request, edge_detection::DetectEdges::Response &response);
        ros::ServiceServer service;

};

