import cv2
from loguru import logger
from ultralytics import YOLO
from ultralytics.engine.results import Results

import cv_bridge
import rospy
from sensor_msgs.msg import CompressedImage


class Globals:
    bridge = cv_bridge.CvBridge()

    seg_model = YOLO("yolov8m-seg.pt")
    pose_model = YOLO("yolov8m-pose.pt")

    yolo_seg_pub: rospy.Publisher
    yolo_pose_pub: rospy.Publisher


# https://docs.ultralytics.com/modes/predict/
def rgb_callback(msg: CompressedImage) -> None:
    img = Globals.bridge.compressed_imgmsg_to_cv2(msg)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    results: list[Results] = Globals.seg_model.predict(img)
    annotated_img = results[0].plot()

    annotated_msg = Globals.bridge.cv2_to_compressed_imgmsg(annotated_img)
    Globals.yolo_seg_pub.publish(annotated_msg)


# https://docs.ultralytics.com/tasks/
def skel_callback(msg: CompressedImage) -> None:
    img = Globals.bridge.compressed_imgmsg_to_cv2(msg)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    results: list[Results] = Globals.pose_model.predict(img)
    annotated_img = results[0].plot()

    annotated_msg = Globals.bridge.cv2_to_compressed_imgmsg(annotated_img)
    Globals.yolo_pose_pub.publish(annotated_msg)


if __name__ == "__main__":
    rospy.init_node("yolov8")
    logger.success("node initialized")

    Globals.yolo_seg_pub = rospy.Publisher(
        "/yolov8/seg/compressed", CompressedImage, queue_size=1
    )
    Globals.yolo_pose_pub = rospy.Publisher(
        "/yolov8/pose/compressed", CompressedImage, queue_size=1
    )

    rospy.Subscriber(
        "/camera/color/image_raw/compressed",
        CompressedImage,
        rgb_callback,
        queue_size=1,
        buff_size=5 * 1024**2,
    )
    # There may be a better way provide multiple callbacks to a subscriber.
    # queue_size and buff_size need to be equal to the previous subscriber's,
    # otherwise bad things will happen
    rospy.Subscriber(
        "/camera/color/image_raw/compressed",
        CompressedImage,
        skel_callback,
        queue_size=1,
        buff_size=5 * 1024**2,
    )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
