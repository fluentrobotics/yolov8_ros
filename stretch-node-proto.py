import cv2
import torch
from loguru import logger
from ultralytics import YOLO
from ultralytics.engine.results import Results

import cv_bridge
import rospy
from sensor_msgs.msg import CompressedImage

from profiler import Profile


class Globals:
    bridge = cv_bridge.CvBridge()

    # TODO: try OpenVINO export on Intel CPU-only machines
    # https://docs.ultralytics.com/modes/export/
    # https://docs.ultralytics.com/integrations/openvino/
    seg_model = YOLO("models/yolov8s-seg.pt")
    pose_model = YOLO("models/yolov8s-pose.pt")

    # if True or not torch.cuda.is_available():
    #     # convert to OpenVino
    #     # TODO: check if the openvino model already exists
    #     seg_model.export(format="openvino")
    #     pose_model.export(format="openvino")

    #     # TODO: de-hardcode these
    #     seg_model = YOLO("models/yolov8s-seg_openvino_model")
    #     pose_model = YOLO("models/yolov8s-pose_openvino_model")

    yolo_seg_pub: rospy.Publisher
    yolo_pose_pub: rospy.Publisher


# https://docs.ultralytics.com/modes/predict/
@Profile(50)
def rgb_callback(msg: CompressedImage) -> None:
    img = Globals.bridge.compressed_imgmsg_to_cv2(msg)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    results: list[Results] = Globals.seg_model.predict(img, verbose=False)
    annotated_img = results[0].plot()

    annotated_msg = Globals.bridge.cv2_to_compressed_imgmsg(annotated_img)
    Globals.yolo_seg_pub.publish(annotated_msg)


# https://docs.ultralytics.com/tasks/
@Profile(50)
def skel_callback(msg: CompressedImage) -> None:
    img = Globals.bridge.compressed_imgmsg_to_cv2(msg)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

    results: list[Results] = Globals.pose_model.predict(img, verbose=False)
    annotated_img = results[0].plot()

    annotated_msg = Globals.bridge.cv2_to_compressed_imgmsg(annotated_img)
    Globals.yolo_pose_pub.publish(annotated_msg)


if __name__ == "__main__":
    rospy.init_node("yolov8")

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

    logger.success("node initialized")

    while not rospy.is_shutdown():
        summary = Profile.summary()
        if summary:
            print(summary, end="\n\n")
        rospy.sleep(1)
