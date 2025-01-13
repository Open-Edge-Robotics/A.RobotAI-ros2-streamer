from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="stream_server",
        description="Stream server namespace for the nodes",
    )

    topic = LaunchConfiguration("topic")
    topic_cmd = DeclareLaunchArgument(
        "topic", default_value="stream", description="Stream server publish topic name"
    )

    port = LaunchConfiguration("port")
    port_cmd = DeclareLaunchArgument(
        "port", default_value="5555", description="TCP port for images streaming"
    )

    image_raw = LaunchConfiguration("image_raw")
    image_raw_cmd = DeclareLaunchArgument(
        "image_raw",
        default_value="/camera/color/image_raw",
        description="Name of the input image topic",
    )

    compressed_image_raw = LaunchConfiguration("compressed_image_raw")
    compressed_image_raw_cmd = DeclareLaunchArgument(
        "compressed_image_raw",
        default_value="/camera/color/image_raw/compressed",
        description="Name of the input compressed image topic",
    )

    image_reliability = LaunchConfiguration("image_reliability")
    image_reliability_cmd = DeclareLaunchArgument(
        "image_reliability",
        default_value="2",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)",
    )

    stream_server_node_cmd = Node(
        package="ros_stream_server",
        executable="stream_server",
        name="stream_server",
        namespace=namespace,
        parameters=[
            {
                "topic": topic,
                "port": port,
                "image_raw": image_raw,
                "compressed_image_raw": compressed_image_raw,
                "image_reliability": image_reliability,
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(topic_cmd)
    ld.add_action(port_cmd)
    ld.add_action(image_raw_cmd)
    ld.add_action(compressed_image_raw_cmd)
    ld.add_action(image_reliability_cmd)
    ld.add_action(namespace_cmd)

    ld.add_action(stream_server_node_cmd)

    return ld
