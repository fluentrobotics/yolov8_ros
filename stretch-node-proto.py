import cv2
import numpy as np
from loguru import logger
from ultralytics import YOLO
from ultralytics.engine.results import Results

import cv_bridge
import rospy
from sensor_msgs.msg import CompressedImage


class Globals:
    model = YOLO("yolov8s.pt")
    yolo_pub: rospy.Publisher


def rgb_callback(msg: CompressedImage) -> None:
    logger.debug("received msg")

    buf = np.frombuffer(msg.data, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
    img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    results: list[Results] = Globals.model.predict(img)

    annotated_img = results[0].plot()
    _, annotated_img_buf = cv2.imencode(".jpg", annotated_img)
    msg.data = annotated_img_buf.tolist()

    Globals.yolo_pub.publish(msg)


if __name__ == "__main__":
    # set up ros node and subscribers
    # rospy.spin or something
    rospy.init_node("yolov8")
    logger.success("node initialized")

    Globals.yolo_pub = rospy.Publisher(
        "/yolov8/compressed", CompressedImage, queue_size=1
    )
    rospy.Subscriber(
        "/camera/color/image_raw/compressed",
        CompressedImage,
        rgb_callback,
        queue_size=1,
        buff_size=5 * 1024**2,
    )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
