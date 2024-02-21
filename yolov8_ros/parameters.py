from dataclasses import dataclass, field

from rclpy.node import Node


@dataclass
class YoloSkeletonRightWristNodeParams:
    yolov8_model_name: str = field(default="yolov8x-pose.pt")
    """
    Basename of the YOLOv8 model to use.

    A list exists here: https://docs.ultralytics.com/tasks/pose/#models
    """

    rgb_sub_topic: str = field(default="/camera/color/image_raw/compressed")
    """sensor_msgs/CompressedImage RGB topic to subscribe to."""

    depth_sub_topic: str = field(default="/camera/aligned_depth_to_color/image_raw")
    """
    sensor_msgs/Image aligned depth topic to subscribe to.

    Images published to this topic must share the same intrinsic matrix as the
    RGB image topic.

    The uncompressed topic is used as there seems to be some difficulty
    subscribing or decoding the compressed depth topic.
    """

    camera_info_sub_topic: str = field(default="/camera/color/camera_info")
    """sensor_msgs/CameraInfo topic to subscribe to."""

    stretch_robot_rotate_image_90deg: bool = field(default=True)
    """
    Whether to rotate the input images 90 degrees clockwise before inference.

    This parameter exists because the Stretch robot's camera is mounted
    sideways, and YOLOv8 does not appear to perform well on rotated images.

    TODO: This could be implemented better, but it works for our use case at the
    time of writing.
    """

    @staticmethod
    def read_from_ros(node: Node) -> "YoloSkeletonRightWristNodeParams":
        params = YoloSkeletonRightWristNodeParams()

        for param_name, default_value in vars(params).items():
            ros_value = node.declare_parameter(param_name, default_value).value
            assert type(ros_value) == type(default_value)
            setattr(params, param_name, ros_value)

        return params
