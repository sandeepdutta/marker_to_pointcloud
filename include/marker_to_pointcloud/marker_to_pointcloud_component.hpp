#ifndef MARKER_TO_POINTCLOUD_COMPONENT_HPP_
#define MARKER_TO_POINTCLOUD_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace marker_to_pointcloud
{

class MarkerToPointCloud : public rclcpp::Node
{
public:
  explicit MarkerToPointCloud(const rclcpp::NodeOptions &options);

private:
  void markerCallback(const visualization_msgs::msg::Marker::SharedPtr marker_msg);
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr marker_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

}  // namespace marker_to_pointcloud


#endif  // MARKER_TO_POINTCLOUD_COMPONENT_HPP_
