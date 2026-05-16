# #!/usr/bin/env python3

# import threading
# import cv2
# from flask import Flask, Response

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge


# app = Flask(__name__)

# latest_frame = None
# lock = threading.Lock()


# class ImageSubscriber(Node):

#     def __init__(self):
#         super().__init__('image_stream_node')

#         self.bridge = CvBridge()

#         self.sub = self.create_subscription(
#             Image,
#             '/image_raw',
#             self.callback,
#             10
#         )

#         self.get_logger().info('Subscribed to /image_raw')


#     def callback(self, msg):

#         global latest_frame

#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

#             with lock:
#                 latest_frame = frame

#         except Exception as e:
#             self.get_logger().error(str(e))


# def gen_frames():

#     global latest_frame

#     while True:

#         with lock:
#             if latest_frame is None:
#                 continue

#             frame = latest_frame.copy()

#         ret, buffer = cv2.imencode('.jpg', frame)

#         if not ret:
#             continue

#         jpg = buffer.tobytes()

#         yield (b'--frame\r\n'
#                b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')


# @app.route('/')
# def video_feed():

#     return Response(
#         gen_frames(),
#         mimetype='multipart/x-mixed-replace; boundary=frame'
#     )


# def ros_spin():

#     rclpy.init()

#     node = ImageSubscriber()

#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':

#     t = threading.Thread(target=ros_spin)
#     t.daemon = True
#     t.start()

#     print('ROS2 + Flask streamer started')

#     app.run(host='0.0.0.0', port=5000)

#!/usr/bin/env python3

import threading
import time

import cv2
import numpy as np
from flask import Flask, Response, jsonify

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


app = Flask(__name__)

latest_frame = None
latest_frame_time = 0.0
image_topic_name = "/image_processed"

frame_lock = threading.Lock()


class RosImageStreamNode(Node):
    def __init__(self):
        super().__init__("ros_stream")

        self.declare_parameter("image_topic", "/image_processed")
        self.declare_parameter("jpeg_quality", 80)

        self.image_topic = self.get_parameter("image_topic").value
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        global image_topic_name
        image_topic_name = self.image_topic

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"✅ ROS stream subscribed to: {self.image_topic}")
        self.get_logger().info("🌐 Flask server should be available on port 5000")

    def image_callback(self, msg):
        global latest_frame
        global latest_frame_time

        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            with frame_lock:
                latest_frame = frame.copy()
                latest_frame_time = time.time()

        except Exception as e:
            self.get_logger().error(f"❌ Failed to convert ROS image: {e}")


def make_waiting_frame():
    frame = np.zeros((480, 640, 3), dtype=np.uint8)

    cv2.putText(
        frame,
        "Waiting for ROS image...",
        (85, 220),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        2
    )

    cv2.putText(
        frame,
        image_topic_name,
        (135, 270),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (180, 180, 180),
        2
    )

    return frame


def get_current_frame():
    with frame_lock:
        if latest_frame is None:
            return None

        return latest_frame.copy()


def generate_mjpeg():
    while True:
        frame = get_current_frame()

        if frame is None:
            frame = make_waiting_frame()

        ok, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        )

        if not ok:
            time.sleep(0.03)
            continue

        jpg = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n"
        )

        time.sleep(0.03)


@app.route("/")
def index():
    return Response(
        generate_mjpeg(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/health")
def health():
    with frame_lock:
        has_frame = latest_frame is not None
        age = None

        if latest_frame_time > 0.0:
            age = round(time.time() - latest_frame_time, 3)

    return jsonify({
        "status": "ok",
        "topic": image_topic_name,
        "has_frame": has_frame,
        "frame_age_sec": age,
        "port": 5000
    })


def ros_thread_main():
    rclpy.init()

    node = RosImageStreamNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    ros_thread = threading.Thread(
        target=ros_thread_main,
        daemon=True
    )

    ros_thread.start()

    print("✅ stream_ros started")
    print("🌐 Video:  http://ROBOT_IP:5000")
    print("🩺 Health: http://ROBOT_IP:5000/health")

    app.run(
        host="0.0.0.0",
        port=5000,
        threaded=True,
        use_reloader=False
    )


if __name__ == "__main__":
    main()