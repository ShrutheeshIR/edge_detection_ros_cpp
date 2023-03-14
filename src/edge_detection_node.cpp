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
    pc_pub = nh_.advertise<sensor_msgs::PointCloud2>("edge_points", 10);
    marker_pub = nh_.advertise<visualization_msgs::MarkerArray> ("viz_marker_array", 0);

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

            cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
            cv::Mat depth_img;
            cv::Mat(cv_ptr -> image).convertTo(depth_img, CV_32FC1, 0.001);


            std::vector<geometry_msgs::Point> edges = srv.response.edge_output.edges_2d;

            int index = 0;
            sensor_msgs::PointCloud2 output;
            typename pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

            visualization_msgs::MarkerArray marker_array;

            for(geometry_msgs::Point edge: edges)
            {
                double depth = depth_img.at<double>((int)edge.y, (int)edge.x);

                // std::cout << depth << ", " << depth_img.at<double>(100,100) << std::endl;
                double x = depth * (edge.x - K.at<double>(0, 2)) / K.at<double>(0, 0);
                double y = depth * (edge.x - K.at<double>(1, 2)) / K.at<double>(1, 1);
                double z = depth;
                // result.push_back(pt);
                // std::cout << pt.x << ", " << pt.y << ", " << pt.z << std::endl;
                cluster->push_back(pcl::PointXYZ(x, y, z));
                index++;
                if(index%5 == 0)
                    marker_array.markers.push_back(place_marker(x, y, z, index));

            }

            pcl::toROSMsg(*cluster.get(),output);
            // output.header.stamp=ros::Time::now();
            output.header.frame_id="camera_color_optical_frame";

            pc_pub.publish(output);
            marker_pub.publish(marker_array);
           

        }    

}

visualization_msgs::Marker EdgeDetectionNode::place_marker(double x, double y, double z, int idx)
{

    visualization_msgs::Marker marker;
    marker.header.frame_id = "root_link";
    // marker.header.frame_id = "camera_depth_optical_frame";
    ros::Time ros_time = ros::Time::now();
    marker.header.stamp = ros_time;
    marker.ns = "marker_edge";
    marker.id = idx;
    // marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 0.5;

    return marker;

    // marker.lifetime = lifetime;
    // marker.frame_locked = frame_locked;
    // marker.text = "This is some text\nthis is a new line\nthis is another line\nand another     adfoije    owijeoiwej\na really really really really really really really really really really long one";
    // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    // marker.mesh_use_embedded_materials = (i > int((count / 12) % 5));

    // marker_pub.publish(marker);

}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "PubSubNode");
    EdgeDetectionNode e_node = EdgeDetectionNode();
    ros::spin();
}