#ifndef MARKER_TO_POINTCLOUD_COMPONENT_HPP_
#define MARKER_TO_POINTCLOUD_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
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
  std::string camera_frame_id_;
  std::string robot_frame_id_;
  float min_z_, max_z_;
  float frame_diff_x_, frame_diff_y_, frame_diff_z_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
};

}  // namespace marker_to_pointcloud


#endif  // MARKER_TO_POINTCLOUD_COMPONENT_HPP_
