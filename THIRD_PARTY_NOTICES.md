# Third-Party Notices

This repository redistributes or adapts components from the projects below.
See the linked licenses for full terms.

| Component | License | Location / notes |
| --- | --- | --- |
| [Apollo](https://github.com/ApolloAuto/apollo) (r3.0 perception) | Apache-2.0 | Core perception algorithms under `src/perception/apollo_perception_ros/` |
| [PCL](https://github.com/PointCloudLibrary/pcl) / pcl_ros | BSD | Vendored under `src/sensor_drivers/perception_pcl/` |
| geometry_msgs (fork) | BSD | `src/common/common_msgs/geometry_msgs/` |
| visualization_msgs (fork) | BSD | `src/common/common_msgs/visualization_msgs/` |
| ROS ecosystem packages | BSD / respective | catkin dependencies declared in `package.xml` files |
| Caffe | BSD | LiDAR CNN segmentation models and build integration |
| OpenCV | BSD | Camera pipeline integration |
| Eigen | MPL-2.0 | Math utilities |

Upstream Apollo license:
https://github.com/ApolloAuto/apollo/blob/master/LICENSE

If you believe a notice is missing, please open an issue or pull request.
