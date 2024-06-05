#include <rclcpp/rclcpp.hpp>
#include "marker_to_pointcloud_component.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions().use_intra_process_comms(true);
  auto node = std::make_shared<marker_to_pointcloud::MarkerToPointCloud>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
