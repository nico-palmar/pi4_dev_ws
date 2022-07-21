from http.server import executable
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    node = Node(
        package='turtlesim',
        namespace="",
        executable="turtlesim_node"
    )

    container = ComposableNodeContainer(
        name='container',
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="software_training_assignment",
                plugin="composition::SpawnTwoTurtles"),
            ComposableNode(
                package="software_training_assignment",
                plugin="composition::clear_turtles"),
            ComposableNode(
                package="software_training_assignment",
                plugin="composition::MoveAction"),
            ComposableNode(
                package="software_training_assignment",
                plugin="composition::MotionReset"),
            ComposableNode(
                package="software_training_assignment",
                plugin="composition::DistanceInfo"),
            ComposableNode(
                package="software_training_assignment",
                plugin="composition::CircularMotion")
        ]
    )

    return launch.LaunchDescription([node, container])