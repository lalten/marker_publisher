#pragma once

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>

#include <visualization_msgs/Marker.h>
#include <marker_publisher/MarkerArray.h>

#include <aruco/aruco.h>

class MarkerPosePublisher {
    aruco::CameraParameters TheCameraParameters;
    aruco::MarkerDetector TheMarkerDetector;
    float markerSizeMeters;
    std::string camera_frame;

public:
    MarkerPosePublisher();

    void callBackColor(const sensor_msgs::ImageConstPtr &, const sensor_msgs::CameraInfoConstPtr& cinfo);

    tf::Transform arucoMarker2Tf(const aruco::Marker &);

    void publish_marker(geometry_msgs::Pose, int, std_msgs::Header);

    aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                         bool useRectifiedParameters);

protected:
    ros::NodeHandle nh_node;
    image_transport::ImageTransport img_transport;
    image_transport::CameraSubscriber sub;
    ros::Publisher markers_pub_visualization;
    ros::Publisher markers_pub_array;
    image_transport::Publisher markers_pub_debug;
    marker_publisher::MarkerArray::Ptr marker_msg_pub;
    tf::TransformBroadcaster br;
    std::map<int, ros::Publisher> posewithcovariancestamped_publishers;
    bool publish_tf;
    double pose_covariance;
};

