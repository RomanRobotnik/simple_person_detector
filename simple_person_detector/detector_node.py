#!/usr/bin/env python3
import math
from typing import List, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class PersonDetectorNode(Node):
    """Detect people in incoming images and republish annotated frames."""

    def __init__(self) -> None:
        """
        Initializes the PersonDetectorNode.

        This constructor sets up the ROS2 node for person detection using OpenCV's HOG descriptor.
        It declares and retrieves parameters, initializes the HOG detector, sets up image conversion utilities,
        and creates ROS2 publishers and subscribers.

        Parameters declared:
            input_topic (str): ROS topic to subscribe for input images. Default is '/image_raw'.
            output_topic (str): ROS topic to publish annotated images. Default is '/person_detector/image_annotated'.
            win_stride (Tuple[int, int]): Window stride for HOG detector. Default is (8, 8).
            padding (Tuple[int, int]): Padding for HOG detector. Default is (16, 16).
            scale (float): Scale factor for image pyramid in detection. Default is 1.05.
            downscale_factor (float): Factor to downscale input images before detection. Default is 1.0.
            detection_interval (float): Minimum interval (seconds) between detections. Default is 0.3.

        Attributes:
            input_topic (str): The input image topic name.
            output_topic (str): The output annotated image topic name.
            win_stride (Tuple[int, int]): Window stride used in detection.
            padding (Tuple[int, int]): Padding used in detection.
            scale (float): Scale factor for detection.
            downscale_factor (float): Downscale factor for input images.
            detection_interval (float): Minimum time between detections.
            hog (cv2.HOGDescriptor): Initialized HOG descriptor for person detection.
            bridge (CvBridge): Utility for converting ROS images to OpenCV format.
            last_detection_time (Optional[float]): Timestamp of last detection.
            last_rects (List[Tuple[int, int, int, int]]): List of last detected bounding boxes.
            subscription: ROS2 subscription to input image topic.
            publisher: ROS2 publisher for annotated output images.
        """
        super().__init__('person_detector_node')

        # Parameters
        self.declare_parameter('input_topic', '/robot/top_ptz_rgbd_camera/color/image_raw')
        self.declare_parameter('win_stride', [8, 8])
        self.declare_parameter('padding', [16, 16])
        self.declare_parameter('scale', 1.05)
        self.declare_parameter('downscale_factor', 1.0)
        self.declare_parameter('detection_interval', 0.3)

        self.input_topic: str = self.get_parameter('input_topic').value
        self.win_stride: Tuple[int, int] = self._tuple_param('win_stride', (8, 8))
        self.padding: Tuple[int, int] = self._tuple_param('padding', (16, 16))
        self.scale: float = float(self.get_parameter('scale').value)
        self.downscale_factor: float = max(float(self.get_parameter('downscale_factor').value), 0.1)
        self.detection_interval: float = max(float(self.get_parameter('detection_interval').value), 0.0)

        # HOG detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # Utilities
        self.bridge = CvBridge()
        self.last_detection_time = None
        self.last_rects: List[Tuple[int, int, int, int]] = []

        # ROS entities
        self.subscription = self.create_subscription(Image, self.input_topic, self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '~/image_annotated', 10)
        self.detection_publisher = self.create_publisher(Bool, '~/detected', 10)
        self.detections_publisher = self.create_publisher(Detection2DArray, '~/detection_array', 10)

        self.get_logger().info(
            f'Person detector ready (input="{self.input_topic}")'
        )

    def _tuple_param(self, name: str, fallback: Tuple[int, int]) -> Tuple[int, int]:
        value = self.get_parameter(name).value
        if isinstance(value, (list, tuple)) and len(value) == 2:
            return int(value[0]), int(value[1])
        return fallback

    def _should_detect(self) -> bool:
        if self.detection_interval <= 0.0:
            return True
        if self.last_detection_time is None:
            return True
        elapsed = self.get_clock().now() - self.last_detection_time
        return elapsed.nanoseconds >= int(self.detection_interval * 1e9)

    def image_callback(self, msg: Image) -> None:
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as err:
            self.get_logger().error(f'Image conversion failed: {err}')
            return

        if self._should_detect():
            processed = cv_image
            scale_back = 1.0
            if not math.isclose(self.downscale_factor, 1.0):
                new_size = (
                    max(1, int(cv_image.shape[1] / self.downscale_factor)),
                    max(1, int(cv_image.shape[0] / self.downscale_factor)),
                )
                processed = cv2.resize(cv_image, new_size, interpolation=cv2.INTER_LINEAR)
                scale_back = cv_image.shape[1] / processed.shape[1]

            rects, _ = self.hog.detectMultiScale(
                processed, winStride=self.win_stride, padding=self.padding, scale=self.scale
            )
            scaled_rects: List[Tuple[int, int, int, int]] = []
            for (x, y, w, h) in rects:
                if not math.isclose(scale_back, 1.0):
                    x = int(x * scale_back)
                    y = int(y * scale_back)
                    w = int(w * scale_back)
                    h = int(h * scale_back)
                scaled_rects.append((x, y, w, h))

            self.last_rects = scaled_rects
            self.last_detection_time = self.get_clock().now()

        for (x, y, w, h) in self.last_rects:
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(cv_image, 'Person', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        detection_msg = Bool(data=bool(self.last_rects))

        detections_array = Detection2DArray()
        detections_array.header = msg.header
        for (x, y, w, h) in self.last_rects:
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = float(x + w / 2.0)
            detection.bbox.center.position.y = float(y + h / 2.0)
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'person'
            hypothesis.hypothesis.score = 1.0
            detection.results.append(hypothesis)

            detections_array.detections.append(detection)

        try:
            annotated = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            annotated.header = msg.header
            self.publisher.publish(annotated)
            self.detection_publisher.publish(detection_msg)
            self.detections_publisher.publish(detections_array)
        except Exception as err:
            self.get_logger().error(f'Image publish failed: {err}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()