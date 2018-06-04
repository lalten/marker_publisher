#include "marker_publisher/marker_pub.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/transport_hints.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

MarkerPosePublisher::MarkerPosePublisher() : nh_node("~"), img_transport(nh_node) {
    // set dictionary and error correction rate
    std::string dict_type;
    nh_node.param<std::string>("dict_type", dict_type, "ALL_DICTS");
    float error_correction_rate; //error correction rate respect to the maximum error correction capability for each dictionary.
    nh_node.param<float>("error_correction_rate", error_correction_rate, 0.6);
    TheMarkerDetector.setDictionary(dict_type, error_correction_rate);

    // set detection mode and minimum marker size
    float markerSizeMin;
    nh_node.param<float>("marker_min", markerSizeMin, 0.01);
    std::string detectionMode;
    nh_node.param<std::string>("detection_mode", detectionMode, "DM_NORMAL");
    aruco::DetectionMode detectionModeEnum = aruco::DetectionMode::DM_NORMAL;
    if (detectionMode == "DM_NORMAL"){
        detectionModeEnum = aruco::DetectionMode::DM_NORMAL;
    } else if (detectionMode == "DM_FAST"){
        detectionModeEnum = aruco::DetectionMode::DM_FAST;
    } else if (detectionMode == "DM_VIDEO_FAST"){
        detectionModeEnum = aruco::DetectionMode::DM_VIDEO_FAST;
    } else {
      ROS_ERROR_STREAM("Unknown detection_mode "<<detectionMode);
      ros::shutdown();
    }
    TheMarkerDetector.setDetectionMode(detectionModeEnum, markerSizeMin);

    // Overwrite the outgoing header frame
    nh_node.param<std::string>("camera_frame", camera_frame, "");

    // Default marker size
    nh_node.param<float>("markerSizeMeters", markerSizeMeters, -1);

    nh_node.param<bool>("publish_tf", publish_tf, true);

    // Set default_transport param to "compressed" in case you're using a rosbag that only contains compressed images
    std::string default_transport;
    nh_node.param<std::string>("default_transport", default_transport, "raw");
    image_transport::TransportHints transport_hints(default_transport);
    sub = img_transport.subscribeCamera("image_raw", 1, &MarkerPosePublisher::callBackColor, this, transport_hints);

    markers_pub_visualization = nh_node.advertise<visualization_msgs::Marker>("Estimated_marker", 1);
    markers_pub_array = nh_node.advertise<marker_publisher::MarkerArray>("MarkerArray", 1);
    markers_pub_debug = img_transport.advertise("debug", 1);

    marker_msg_pub = marker_publisher::MarkerArray::Ptr(new marker_publisher::MarkerArray());
}

void MarkerPosePublisher::callBackColor(const sensor_msgs::ImageConstPtr &msg, const sensor_msgs::CameraInfoConstPtr& cinfo) {

    // If camera parameters are not defined yet, read them
    if(!TheCameraParameters.isValid())
    {
        // Check if camera_info msg actually contains calibration information
        if(cinfo->K.at(0) == 0.0) // "clients may assume that K[0] == 0.0 indicates an uncalibrated camera"
        {
            ROS_ERROR_STREAM("Camera is uncalibrated!");
            ros::shutdown();
        }
        bool useRectifiedImages;
        nh_node.param<bool>("image_is_rectified", useRectifiedImages, true);
        TheCameraParameters = rosCameraInfo2ArucoCamParams(*cinfo, useRectifiedImages);
    }

    // Convert to grayscale (otherwise Aruco will do internally)
    // Will share instead of copy if input image already is mono8 encoded
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // copy stamp, header, seq. Overwrite frame_id if configured.
    std_msgs::Header msg_header = msg->header;
    if(camera_frame != "")
    {
      msg_header.frame_id = camera_frame;
    }

    std::vector<aruco::Marker> detected_markers = TheMarkerDetector.detect(cv_ptr->image);

    marker_msg_pub->markers.clear();
    marker_msg_pub->markers.resize(detected_markers.size());
    marker_msg_pub->header = msg_header;

    for (size_t i = 0; i < detected_markers.size(); ++i) {
        std::ostringstream o;
        int markerId = detected_markers[i].id;
        o << "marker_" << markerId;
        std::string o_str = o.str();

        float marker_size;
        nh_node.param<float>(o_str, marker_size, markerSizeMeters); // overwrite size of e.g. marker_0

        detected_markers[i].calculateExtrinsics(marker_size, TheCameraParameters.CameraMatrix,
                                                TheCameraParameters.Distorsion, false);

        tf::Transform object_transform = arucoMarker2Tf(detected_markers[i]);

        if(publish_tf)
        {
          br.sendTransform(tf::StampedTransform(object_transform, msg_header.stamp, msg_header.frame_id, o_str));
        }

        geometry_msgs::Pose marker_pose_data;

        const tf::Vector3 marker_origin = object_transform.getOrigin();
        marker_pose_data.position.x = marker_origin.getX();
        marker_pose_data.position.y = marker_origin.getY();
        marker_pose_data.position.z = marker_origin.getZ();

        tf::Quaternion marker_quaternion = object_transform.getRotation();
        marker_pose_data.orientation.x = marker_quaternion.getX();
        marker_pose_data.orientation.y = marker_quaternion.getY();
        marker_pose_data.orientation.z = marker_quaternion.getZ();
        marker_pose_data.orientation.w = marker_quaternion.getW();

        publish_marker(marker_pose_data, markerId, msg_header);

        //Publish markers
        marker_publisher::Marker &marker_i = marker_msg_pub->markers.at(i);
        marker_i.idx = markerId;
        tf::Transform transform = arucoMarker2Tf(detected_markers[i]);
        tf::poseTFToMsg(transform, marker_i.pose.pose);

        // Publish PoseWithCovarianceStamped on topic poses/marker_i
        if(posewithcovariancestamped_publishers.count(markerId) == 0)
        {
          // If it doesn't exist yet, create this publisher
          std::stringstream ss;
          ss << "poses/marker_" << (int) markerId;
          posewithcovariancestamped_publishers[markerId] = nh_node.advertise<geometry_msgs::PoseWithCovarianceStamped>(ss.str(), 1);
        }
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = msg_header;
        pose_msg.pose = marker_i.pose;
        posewithcovariancestamped_publishers[markerId].publish(pose_msg);
    }

    // only publish if markers detected
    if(detected_markers.size() > 0) {
        markers_pub_array.publish(marker_msg_pub);
    }

    // publish debug image with markers
    if(markers_pub_debug.getNumSubscribers() > 0)
    {
      cv_bridge::CvImagePtr debug_img_msg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        for(auto &m : detected_markers)
        {
            m.draw(debug_img_msg->image, cv::Scalar(0, 0, 255), 3, true, true);
//          aruco::CvDrawingUtils::draw3dCube(debug_img_msg->image, m, TheCameraParameters);
            aruco::CvDrawingUtils::draw3dAxis(debug_img_msg->image, m, TheCameraParameters);
        }
        markers_pub_debug.publish(debug_img_msg->toImageMsg());
    }
}


tf::Transform MarkerPosePublisher::arucoMarker2Tf(const aruco::Marker &marker) {
    cv::Mat marker_rotation;
    cv::Rodrigues(marker.Rvec, marker_rotation);
    cv::Mat marker_translation = marker.Tvec;

    tf::Matrix3x3 marker_tf_rot(marker_rotation.at<float>(0, 0), marker_rotation.at<float>(0, 1),
                                marker_rotation.at<float>(0, 2),
                                marker_rotation.at<float>(1, 0), marker_rotation.at<float>(1, 1),
                                marker_rotation.at<float>(1, 2),
                                marker_rotation.at<float>(2, 0), marker_rotation.at<float>(2, 1),
                                marker_rotation.at<float>(2, 2));

    tf::Vector3 marker_tf_tran(marker_translation.at<float>(0, 0),
                               marker_translation.at<float>(1, 0),
                               marker_translation.at<float>(2, 0));

    return tf::Transform(marker_tf_rot, marker_tf_tran);
}

void MarkerPosePublisher::publish_marker(geometry_msgs::Pose marker_pose, int marker_id, std_msgs::Header msg_header) {
    visualization_msgs::Marker marker;
    marker.header = msg_header;
    marker.ns = "basic_shapes";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = marker_pose;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.01;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(0.1);

    markers_pub_visualization.publish(marker);
}

aruco::CameraParameters MarkerPosePublisher::rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                                bool useRectifiedParameters)
{
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    cv::Mat distorsionCoeff(4, 1, CV_64FC1);
    cv::Size size(cam_info.height, cam_info.width);

    if ( useRectifiedParameters )
    {
        cameraMatrix.setTo(0);
        cameraMatrix.at<double>(0,0) = cam_info.P[0];   cameraMatrix.at<double>(0,1) = cam_info.P[1];   cameraMatrix.at<double>(0,2) = cam_info.P[2];
        cameraMatrix.at<double>(1,0) = cam_info.P[4];   cameraMatrix.at<double>(1,1) = cam_info.P[5];   cameraMatrix.at<double>(1,2) = cam_info.P[6];
        cameraMatrix.at<double>(2,0) = cam_info.P[8];   cameraMatrix.at<double>(2,1) = cam_info.P[9];   cameraMatrix.at<double>(2,2) = cam_info.P[10];

        for(int i=0; i<4; ++i)
            distorsionCoeff.at<double>(i, 0) = 0;
    }
    else
    {
        for(int i=0; i<9; ++i)
            cameraMatrix.at<double>(i%3, i-(i%3)*3) = cam_info.K[i];

        if(cam_info.D.size() == 4)
        {
            for(int i=0; i<4; ++i)
                distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
        }
        else
        {
            ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
            for(int i=0; i<4; ++i)
                distorsionCoeff.at<double>(i, 0) = 0;
        }
    }

    return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}




















