#include "edge_detection/edge_detection_node.h"

EdgeDetectionNode :: EdgeDetectionNode()
{

    image_sub.subscribe(nh_, "/camera/color/image_raw", 1);
    depth_image_sub.subscribe(nh_, "/camera/depth/image_rect_raw", 1);
    info_sub = nh_.subscribe("/camera/depth/camera_info", 1, &EdgeDetectionNode::camera_info_callback, this);


    K = cv::Mat::eye(cv::Size(3,3), CV_64FC1);

    sync_.reset(new Sync(MySyncPolicy(10), image_sub, depth_image_sub));
    sync_->registerCallback(boost::bind(&EdgeDetectionNode::message_callback, this, _1, _2));
    client = nh_.serviceClient<edge_detection::DetectEdges>("detect_edges");
    
    edges_image_pub = nh_.advertise<sensor_msgs::Image>("edges_image", 10);
    pc_pub = nh_.advertise<sensor_msgs::PointCloud2>("pc_pub", 10);

}

void EdgeDetectionNode::camera_info_callback(const sensor_msgs::CameraInfoConstPtr& camInfo_msg) {
    cv::Mat intrK(3, 3, CV_64FC1, (void *) camInfo_msg->K.data());
    K = intrK.clone();
}

void EdgeDetectionNode::message_callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::ImageConstPtr& depth_image)
{
    edge_detection::DetectEdges srv;
    srv.request.img = *image;
    if (client.call(srv))
        {
            sensor_msgs::Image output_image = srv.response.edge_output.img;
            edges_image_pub.publish(output_image);

            // cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
            // cv::Mat depth_img;
            // cv::Mat(cv_ptr -> image).convertTo(depth_img, CV_32FC1, 0.001);


            // std::vector<geometry_msgs::Point> edges = srv.response.edge_output.edges_2d;
            // // std::cout << edges.at<double>(4).x << std::endl;
            // // cv::Mat edges_mat = cv::Mat(edges);

            // // cv::Mat out_3d_points;
            // // cv::rgbd::depthTo3dSparse(cv_img, edges_mat, cv::Mat::eye(cv::Size((3,3))), out_3d_points);

            // int index = 0;
            // cv::Mat edges_mat(cv::Size(2, edges.size()), CV_16UC1);
            // // std::cout << "Total size : " << edges_mat.rows << ", " << edges_mat.cols << std::endl;

            // // std::vector<cv::Point3d> result;
            // // sensor_msgs::PointCloud2 pc;
            // sensor_msgs::PointCloud2 output;

            // typename pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

            // for(geometry_msgs::Point edge: edges)
            // {
            //     // cv::Point3d pt;
                
            //     double depth = depth_img.at<double>((int)edge.y, (int)edge.x);

            //     // std::cout << depth << ", " << depth_img.at<double>(100,100) << std::endl;
            //     double x = depth * (edge.x - K.at<double>(0, 2)) / K.at<double>(0, 0);
            //     double y = depth * (edge.x - K.at<double>(1, 2)) / K.at<double>(1, 1);
            //     double z = depth;
            //     // result.push_back(pt);
            //     // std::cout << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
            //     cluster->push_back(pcl::PointXYZ(x, y, z));

            // }

            // pcl::toROSMsg(*cluster.get(),output);
            // // output.header.stamp=ros::Time::now();
            // output.header.frame_id="base_camera_link_1";

            // pc_pub.publish(output);
           

        }    

}


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "PubSubNode");
    EdgeDetectionNode e_node = EdgeDetectionNode();
    ros::spin();
}