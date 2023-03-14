#include "edge_detection/DetectEdges.h"
#include "edge_detection/edge_detection_service.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <vector>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class EdgeDetectionNode
{
    ros::NodeHandle nh_;
    public:
        EdgeDetectionNode();
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<sensor_msgs::Image> depth_image_sub;
        ros::Subscriber info_sub;
        ros::ServiceClient client;

        cv_bridge::CvImagePtr cv_ptr;


        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> sync_;

        // void edge_detection_image_callback(const sensor_msgs::ImageConstPtr& msg);
        void message_callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& depth_image);
        void camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camInfo_msg);



        // void message_callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::CameraInfoConstPtr& cam_info);
        
        ros::Publisher edges_image_pub;
        ros::Publisher pc_pub;

        cv::Mat K; //camera intrinsics

};

