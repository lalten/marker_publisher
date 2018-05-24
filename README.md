# marker_publisher
ros-package for [Aruco 3.0][1] released recently, by the end of 2017.

It detects all the markers that belong to a particular dictionary that has been specified in *dict_type* with their sizes and publish them in MarkerArray message and as tranformations [tf][2]

### size configuration
I suppose that all the markers have the same size which you can modify in the launch file in *markerSizeMeters* param. Now, if you have another marker(s) with different size then you have to specify that separately like so

    <param name="marker_i" value="m_i" /> <!-- Marker_id=i Size meter-->

Where i with the *id* of the marker and *m_i* its size.

### Prerequisites
* Calibration:

You need to calibrate the camera and make sure the camera_info is published (usually via [image_transport](http://ros.org/wiki/image_transport) and [camera_info_manager](http://wiki.ros.org/camera_info_manager)).

* Install Aruco 3:

[ArUco: a minimal library for Augmented Reality applications based on OpenCV](http://www.uco.es/investiga/grupos/ava/node/26)

You can have rosdep install Aruco. It might ask you for your password to install Aruco to `/usr/local`.
```
sudo sh -c 'echo "yaml https://rawgit.com/lalten/marker_publisher/master/rosdep/aruco.yaml " > /etc/ros/rosdep/sources.list.d/15-marker_publisher.list'
rosdep update
rosdep install --from-paths . --ignore-src --rosdistro=$ROS_DISTRO -y
```

### ROS API

#### Messages

 * marker_publisher/Marker.msg

        uint32 idx
        geometry_msgs/PoseWithCovariance pose

 * marker_publisher/MarkerArray.msg

        Header header
        marker_publisher/Marker[] markers

#### Parameters

Required:
 * `markerSizeMeters`: Default marker size

Optional:
 * `dict_type`: What marker dictionary to use (default: `ALL_DICTS`)
 * `error_correction_rate`: Default 0.6
 * `marker_min`: Minimum marker size (default 0.01)
 * `detection_mode`: default: `DM_NORMAL`
 * `camera_frame`: If set, overwrite the outgoing header frame with this (default: unset, will use image's header frame)
 * `default_transport`: Transport hint to use for incoming messages. Default: `raw`
 * `marker_i`: Individual marker size, see above

[1]: https://sourceforge.net/projects/aruco/files/3.0.0/
[2]: http://wiki.ros.org/tf
