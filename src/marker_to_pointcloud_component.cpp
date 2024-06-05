#include "marker_to_pointcloud/marker_to_pointcloud_component.hpp"

namespace marker_to_pointcloud
{

MarkerToPointCloud::MarkerToPointCloud(const rclcpp::NodeOptions &options)
: Node("marker_to_pointcloud_node", options)
{
  marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
    "/marker_topic", 10, std::bind(&MarkerToPointCloud::markerCallback, this, std::placeholders::_1));
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_topic", 10);
}

void MarkerToPointCloud::markerCallback(const visualization_msgs::msg::Marker::SharedPtr marker_msg)
{
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pointcloud_msg.header = marker_msg->header;
  pointcloud_msg.height = 1;
  pointcloud_msg.width = marker_msg->points.size();
  pointcloud_msg.is_dense = false;
  pointcloud_msg.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");

  for (const auto &point : marker_msg->points)
  {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  pointcloud_pub_->publish(pointcloud_msg);
}

}  // namespace marker_to_pointcloud
RCLCPP_COMPONENTS_REGISTER_NODE(marker_to_pointcloud::MarkerToPointCloud)