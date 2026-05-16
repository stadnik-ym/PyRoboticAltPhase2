#!/usr/bin/env python3

import threading
import cv2
from flask import Flask, Response

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


app = Flask(__name__)

latest_frame = None
lock = threading.Lock()


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_stream_node')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/image_raw',
            self.callback,
            10
        )

        self.get_logger().info('Subscribed to /image_raw')


    def callback(self, msg):

        global latest_frame

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            with lock:
                latest_frame = frame

        except Exception as e:
            self.get_logger().error(str(e))


def gen_frames():

    global latest_frame

    while True:

        with lock:
            if latest_frame is None:
                continue

            frame = latest_frame.copy()

        ret, buffer = cv2.imencode('.jpg', frame)

        if not ret:
            continue

        jpg = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')


@app.route('/')
def video_feed():

    return Response(
        gen_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


def ros_spin():

    rclpy.init()

    node = ImageSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    t = threading.Thread(target=ros_spin)
    t.daemon = True
    t.start()

    print('ROS2 + Flask streamer started')

    app.run(host='0.0.0.0', port=5000, threaded= True)
