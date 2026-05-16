from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ---------------- CAMERA CAPTURE / IMX708 / LIBCAMERA ----------------
        Node(
            package='robot_vision',
            executable='camera_capture_node',
            name='camera_capture_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device': '/dev/video0',

                'width': 640,
                'height': 480,
                'fps': 20.0,

                'image_topic': '/image_raw',
                'status_topic': '/camera/status',

                # auto = спробує libcamera, потім v4l2_mjpg, потім v4l2_raw
                'gst_source': 'auto',

                # Якщо треба буде вручну задати pipeline — сюди.
                # Поки лишаємо порожнім.
                'gstreamer_pipeline': '',

                'retry_period_sec': 5.0,
                'warmup_read_attempts': 20,
                'warmup_sleep_sec': 0.05,

                'publish_status': True,

                'flip_horizontal': False,
                'flip_vertical': False,
            }]
        ),
                # ---------------- ARUCO DETECTOR ----------------
        Node(
            package="robot_vision",
            executable="aruco_detector_node",
            name="aruco_detector_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "input_image_topic": "/image_raw",
                "status_topic": "/aruco/status",

                # У тебе раніше використовувався саме ORIGINAL.
                "dictionary": "DICT_ARUCO_ORIGINAL",

                # -1 = приймати будь-який ArUco marker.
                # Якщо треба тільки конкретний id — постав число, наприклад 23.
                "target_id": -1,

                "process_scale": 1.0,
                "enable_clahe": True,
                "enable_sharpen": True,

                "adaptive_thresh_win_min": 5,
                "adaptive_thresh_win_max": 35,
                "adaptive_thresh_win_step": 10,

                "min_marker_perimeter_rate": 0.03,
                "max_marker_perimeter_rate": 4.0,
                "polygonal_approx_accuracy_rate": 0.03,

                'publish_debug_image': False,

                "log_period_sec": 1.0,
            }],
        ),

        # ---------------- VISION DEBUG PROCESSOR ----------------
        Node(
            package="robot_vision",
            executable="vision_processor_node",
            name="vision_processor_node",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "input_image_topic": "/image_raw",
                "aruco_status_topic": "/aruco/status",
                "processed_image_topic": "/image_processed",

                "center_deadzone_px": 60.0,
                "status_timeout_sec": 0.5,

                # 1 = публікувати кожен кадр.
                # 2 = кожен другий кадр, якщо Raspberry Pi буде навантажений.
                "publish_every_n": 1,
            }],
        ),

        # ---------------- MISSION MASTER ----------------
        Node(
            package="robot_vision",
            executable="mission_master_node",
            name="mission_master",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "aruco_status_topic": "/aruco/status",

                # ВАЖЛИВО:
                # Не напряму в мотори, а через safety chain.
                "cmd_topic": "/cmd_vel_raw",
                "buzzer_topic": "/buzzer_signal",

                # -1 = будь-який marker.
                # Якщо треба конкретний — постав той самий id, що і в aruco_detector.
                "target_id": -1,

                "control_rate_hz": 20.0,
                "status_log_period_sec": 1.0,

                # ---------------- CENTERING ----------------
                "center_deadzone_px": 60.0,
                "aruco_turn_kp": 0.0025,
                "max_track_angular": 0.35,

                # Якщо робот повертає не в той бік — постав True.
                "reverse_turn_sign": False,

                # ---------------- APPROACH ----------------
                # Якщо робот їде занадто швидко — зменшуй до 0.04.
                "approach_speed": 0.06,
                "min_approach_speed": 0.025,

                # Головний параметр зупинки по розміру маркера.
                # Якщо зупиняється далеко — збільшуй.
                # Якщо під'їжджає занадто близько — зменшуй.
                "marker_stop_width": 390.0,
                "reached_confirm_frames": 3,

                "allow_forward_while_turning": True,
                "forward_turn_offset_limit_px": 230.0,

                # ---------------- LOST MARKER ----------------
                "aruco_status_timeout_sec": 0.35,
                "lost_stop_sec": 0.25,

                # ---------------- SECTOR SEARCH ----------------
                # Пошук не постійним крученням,
                # а режимом: повернувся -> зупинився -> аналізує кадр.
                "search_turn_speed": 0.28,
                "search_turn_sec": 1.0,
                "search_hold_sec": 0.35,

                # ---------------- SMOOTHING ----------------
                "cmd_smoothing_alpha": 0.55,
            }],
        ),
    ])