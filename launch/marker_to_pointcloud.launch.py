from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes,  Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    container_name = "m2pc_container"
    container = Node(
        name=container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen')
    ns = DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the node'
        )
    m2pc = LoadComposableNodes(
            target_container=container_name,
            composable_node_descriptions=[
                ComposableNode(
                    package='marker_to_pointcloud',
                    plugin='marker_to_pointcloud::MarkerToPointCloud',
                    name='marker_to_pointcloud',
                    namespace=LaunchConfiguration('namespace'),
                    remappings=[
                        ('/marker_topic', '/nvblox_node/occupancy_layer'),
                        ('/pointcloud_topic', '/output_pointcloud_topic')
                    ]
                ),
            ],
        )
    return LaunchDescription([
        container,        
        ns,
        m2pc
    ])
