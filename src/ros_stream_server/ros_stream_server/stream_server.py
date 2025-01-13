import rclpy
import zmq
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image, CompressedImage


class StreamServer(Node):
    def __init__(self):
        super().__init__("stream_server")  # type: ignore
        self.get_logger().info("Stream server is starting...")

        # Set params
        self.declare_parameter("namespace", "default")
        self.declare_parameter("topic", "stream")
        self.declare_parameter("port", 5555)
        self.declare_parameter("image_raw", "/camera/color/image_raw")
        self.declare_parameter(
            "compressed_image_raw", "/camera/color/image_raw/compressed"
        )

        self.namespace = (
            self.get_parameter("namespace").get_parameter_value().string_value
        )
        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        self.image_raw = (
            self.get_parameter("image_raw").get_parameter_value().string_value
        )
        self.compressed_image_raw = (
            self.get_parameter("compressed_image_raw")
            .get_parameter_value()
            .string_value
        )

        # Set QoS profile
        self.declare_parameter(
            "qos_profile",
            QoSReliabilityPolicy.BEST_EFFORT,
        )
        qos_profile = QoSProfile(
            reliability=self.get_parameter("qos_profile")
            .get_parameter_value()
            .integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Set zeromq
        context = zmq.Context()

        self.socket = context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.bind(f"tcp://*:{self.port}")

        self.compressed_socket = context.socket(zmq.PUB)
        self.compressed_socket.setsockopt(zmq.CONFLATE, 1)
        self.compressed_socket.bind(f"tcp://*:{self.port+1}")

        self.get_logger().info(
            f"Stream server is listening on port {self.port} and port {self.port+1} [compressed]..."
        )

        # Set cv2bridge
        self.cv_bridge = CvBridge()

        # Set subscription
        self.sub = self.create_subscription(
            msg_type=Image,
            topic=self.image_raw,
            callback=self.send_stream,
            qos_profile=qos_profile,
        )

        self.compressed_sub = self.create_subscription(
            msg_type=CompressedImage,
            topic=self.compressed_image_raw,
            callback=self.send_compressed_stream,
            qos_profile=qos_profile,
        )

    def send_stream(self, data: Image) -> None:
        image = self.cv_bridge.imgmsg_to_cv2(data)

        if image is None:
            self.get_logger().error("Image is None")
            return

        byte_data = image.tobytes()
        self.socket.send(byte_data)
        self.get_logger().info(f"Sending stream...")

    def send_compressed_stream(self, data: CompressedImage) -> None:
        image = self.cv_bridge.compressed_imgmsg_to_cv2(data)

        if image is None:
            self.get_logger().error("Image is None")
            return

        byte_data = image.tobytes()
        self.compressed_socket.send(byte_data)
        self.get_logger().info(f"Sending compressed stream...")


def main(args=None):
    rclpy.init(args=args)
    node = StreamServer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
