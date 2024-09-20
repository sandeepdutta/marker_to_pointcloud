#include "marker_to_pointcloud/marker_to_pointcloud_component.hpp"

namespace marker_to_pointcloud
{

MarkerToPointCloud::MarkerToPointCloud(const rclcpp::NodeOptions &options)
: Node("marker_to_pointcloud_node", options),tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())) {

	transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	min_z_ = this->declare_parameter("min_z", -std::numeric_limits<float>::infinity());
	max_z_ = this->declare_parameter("max_z", std::numeric_limits<float>::infinity());
	camera_frame_id_ = this->declare_parameter("camera_frame_id", "camera_link");
	robot_frame_id_ = this->declare_parameter("target_frame_id", "base_link");
	marker_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
	  "/marker_topic", 10, std::bind(&MarkerToPointCloud::markerCallback, this, std::placeholders::_1));
	pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_topic", 10);
	
	tf2::Transform transform;
	while (true) {
		try {
			// wait for the transform to be available
			auto gmt = tf_buffer_->lookupTransform(robot_frame_id_, camera_frame_id_, tf2::TimePointZero);
			tf2::fromMsg(gmt.transform, transform);
			break;
		} catch (tf2::TransformException &ex) {
			RCLCPP_ERROR(this->get_logger(), "Transform unavailable waiting: %s", ex.what());
			rclcpp::sleep_for(std::chrono::milliseconds(100));
		}
	}
	// calculate the difference between the camera and robot frames
	frame_diff_x_ = transform.getOrigin().getX();
	frame_diff_y_ = transform.getOrigin().getY();
	frame_diff_z_ = transform.getOrigin().getZ();

	RCLCPP_INFO(this->get_logger(), "MarkerToPointCloud has been started. Translation from %s to %s: x: %f, y: %f, z: %f",
				camera_frame_id_.c_str(), robot_frame_id_.c_str(), frame_diff_x_, frame_diff_y_, frame_diff_z_);

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
	for (const auto &point : marker_msg->points) {		
		if (point.z < min_z_ || point.z > max_z_) { 
			// Skip points outside the z range			
			RCLCPP_DEBUG(this->get_logger(),"Point reject out of range  %f, min (%f) , max (%f)", point.z, min_z_, max_z_);			
			continue;		
		}		
		*iter_x = point.x - frame_diff_x_;		
		*iter_y = point.y - frame_diff_y_;		
		*iter_z = point.z - frame_diff_z_;		
		++iter_x;		
		++iter_y;		
		++iter_z;
	}
	RCLCPP_DEBUG(this->get_logger(), "Publishing point cloud message.");
	// transform the pointcloud to the target frame
	try {
		// transform the pointcloud to the camera frame
	  	tf2::doTransform(pointcloud_msg, pointcloud_msg, tf_buffer_->lookupTransform(
	     				 robot_frame_id_, pointcloud_msg.header.frame_id, tf2::TimePointZero));
	} catch (tf2::TransformException &ex) {
	  	RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
	  	return;
	}
	pointcloud_msg.header.frame_id = robot_frame_id_;
	// and publish the pointcloud
	pointcloud_pub_->publish(pointcloud_msg);
}

}  // namespace marker_to_pointcloud
RCLCPP_COMPONENTS_REGISTER_NODE(marker_to_pointcloud::MarkerToPointCloud)