from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---------------- LIBCAMERA / GSTREAMER ENV ----------------
        # Потрібно, щоб ROS-ноди брали нову libcamera 0.7 з /usr/local,
        # а не стару системну libcamera 0.2.
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=[
                '/usr/local/lib/aarch64-linux-gnu:',
                EnvironmentVariable('LD_LIBRARY_PATH', default_value='')
            ]
        ),

        SetEnvironmentVariable(
            name='GST_PLUGIN_PATH',
            value=[
                '/usr/local/lib/aarch64-linux-gnu/gstreamer-1.0:',
                EnvironmentVariable('GST_PLUGIN_PATH', default_value='')
            ]
        ),

        SetEnvironmentVariable(
            name='LIBCAMERA_DATA_DIR',
            value='/usr/local/share/libcamera'
        ),

        SetEnvironmentVariable(
            name='LIBCAMERA_IPA_MODULE_PATH',
            value='/usr/local/lib/aarch64-linux-gnu/libcamera/ipa'
        ),

        # ---------------- LD06 LIDAR ----------------
        Node(
            package='robot_bringup',
            executable='ld06_node',
            name='ld06_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 230400,
                'frame_id': 'laser_frame',

                'scan_topic': '/scan',
                'front_distance_topic': '/lidar/front_distance',
                'closest_topic': '/lidar/closest',

                'publish_rate_hz': 40.0,

                'range_min_m': 0.05,
                'range_max_m': 12.0,

                'min_confidence': 0,
                'angle_resolution_deg': 1.0,

                'angle_offset_deg': 0.0,
                'invert_angle_direction': False,

                'front_min_deg': -45.0,
                'front_max_deg': 45.0,

                'point_max_age_sec': 0.35,
                'front_min_points': 1,
                'front_hold_sec': 0.8,
                'front_filter_window': 3,
            }]
        ),

        # ---------------- CAMERA CAPTURE ----------------
        # УВАГА:
        # Більше НЕ відкриваємо /dev/video4 напряму.
        # CameraCaptureNode тепер має відкривати GStreamer pipeline:
        # libcamerasrc -> NV12 1280x720 -> BGR 640x480 -> appsink
        Node(
            package='camera',
            executable='camera_capture_node',
            name='camera_capture_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'capture_width': 1280,
                'capture_height': 720,

                'width': 800,
                'height': 480,
                'fps': 30.0,

                'image_topic': '/image_raw',
            }]
        ),

        # ---------------- VISION PROCESSOR ----------------
        Node(
            package='camera',
            executable='vision_processor_node',
            name='vision_processor_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'input_image_topic': '/image_raw',
                'processed_image_topic': '/image_processed',

                # ВАЖЛИВО: команди йдуть через safety,
                # тобто vision_processor публікує НЕ напряму в мотори.
                'cmd_topic': '/cmd_vel_raw',
                'buzzer_topic': '/buzzer_signal',

                'log_threshold': 150,
                'marker_stop_width': 230.0,

                'turn_kp': 0.004,
                'center_deadzone_px': 30,

                'search_linear_speed': 0.05,
                'forward_speed': 0.15,
            }]
        ),

        # ---------------- SAFETY FILTER ----------------
        Node(
            package='robot_bringup',
            executable='lidar_safety',
            name='lidar_safety',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'input_cmd_topic': '/cmd_vel_raw',
                'output_cmd_topic': '/cmd_vel',
                'front_distance_topic': '/lidar/front_distance',

                'cmd_timeout_sec': 0.5,
                'lidar_timeout_sec': 0.7,

                'stop_distance_m': 0.35,
                'clear_distance_m': 0.42,

                'allow_rotation_when_blocked': True,
                'allow_reverse_when_blocked': True,

                'max_linear_x': 0.35,
                'max_angular_z': 1.5,

                'enable_slowdown': False,
                'slowdown_distance_m': 0.80,
                'min_slowdown_factor': 0.25,

                'invalid_front_timeout_sec': 0.6,
            }]
        ),

        # ---------------- MOTOR NODE ----------------
        Node(
            package='diff_drive_l298n',
            executable='diff_drive_node',
            name='motor_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'cmd_vel_topic': '/cmd_vel',
            }]
        ),

        # ---------------- ROS STREAMER ----------------
        Node(
            package='stream_ros',
            executable='ros_stream',
            name='ros_stream',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # Поки краще дивитись сирий потік.
                # Коли vision_processor стабільно працює — можна змінити на /image_processed.
                'image_topic': '/image_raw',
                # 'image_topic': '/image_processed',

                'image_quality': 95,
            }]
        ),

    ])