import rclpy
import zmq
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from sensor_msgs.msg import Image, CompressedImage

from .services.stream_predict_service import StreamPredictService


class StreamServer(Node):
    def __init__(self):
        super().__init__(node_name="stream_server")
        self.get_logger().info("Stream server is starting...")

        # Set params
        self.declare_parameter("namespace", "default")
        self.declare_parameter("port", 5555)
        self.declare_parameter("image_raw", "/camera/color/image_raw")
        self.declare_parameter(
            "compressed_image_raw", "/camera/color/image_raw/compressed"
        )

        self.namespace = (
            self.get_parameter("namespace").get_parameter_value().string_value
        )
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

        self.predict_socket = context.socket(zmq.PUB)
        self.predict_socket.setsockopt(zmq.CONFLATE, 1)
        self.predict_socket.bind(f"tcp://*:{self.port+2}")

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

        self.predict_svc = StreamPredictService(
            logger=self.get_logger(),
        )

        self.predict_sub = self.create_subscription(
            msg_type=CompressedImage,
            topic=self.compressed_image_raw,
            callback=self.send_predict_stream,
            qos_profile=qos_profile,
        )

    def send_stream(self, data: Image) -> None:
        try:
            image = self.cv_bridge.imgmsg_to_cv2(data)

            if image is None or not isinstance(image, np.ndarray):
                self.get_logger().error("Image is None")
                return

            byte_data = image.tobytes()
            self.socket.send(byte_data)
            self.get_logger().debug(f"Sending stream: {len(byte_data)} bytes...")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            return

    def send_compressed_stream(self, data: CompressedImage) -> None:
        try:
            image = self.cv_bridge.compressed_imgmsg_to_cv2(data)

            if image is None or not isinstance(image, np.ndarray):
                self.get_logger().error("Image is None")
                return

            byte_data = image.tobytes()
            self.compressed_socket.send(byte_data)
            self.get_logger().debug(
                f"Sending compressed stream: {len(byte_data)} bytes..."
            )

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            return

    def send_predict_stream(self, data: CompressedImage) -> None:
        try:
            image = self.cv_bridge.compressed_imgmsg_to_cv2(data)
            image = self.predict_svc.predict(
                frame=image,
            )

            if image is None or not isinstance(image, np.ndarray):
                self.get_logger().error("Image is None")
                return

            byte_data = image.tobytes()
            self.predict_socket.send(byte_data)
            self.get_logger().debug(
                f"Sending compressed stream: {len(byte_data)} bytes..."
            )

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")
            return

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")
            return


def main(args=None):
    rclpy.init(args=args)
    node = StreamServer()

    try:
        rclpy.spin(node)
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
