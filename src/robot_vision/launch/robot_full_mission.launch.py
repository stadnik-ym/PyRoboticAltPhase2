from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # =========================================================
        # 1. LD06 LIDAR
        # =========================================================
        Node(
            package='robot_bringup',
            executable='ld06_node',
            name='ld06_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baudrate': 230400,

                'scan_topic': '/scan',
                'front_distance_topic': '/lidar/front_distance',
                'closest_topic': '/lidar/closest',

                'frame_id': 'laser_frame',
                'publish_rate_hz': 40.0,

                'range_min_m': 0.05,
                'range_max_m': 12.0,

                'angle_resolution_deg': 1.0,
                'angle_offset_deg': 0.0,
                'invert_angle_direction': False,

                'front_min_deg': -45.0,
                'front_max_deg': 45.0,

                'immediate_front_publish': True,
            }]
        ),

        # =========================================================
        # 2. CAMERA CAPTURE
        # ВАЖЛИВО: fps тут НЕ передаємо, щоб не було конфлікту INTEGER/DOUBLE.
        # camera_capture_node сама бере default fps=20.
        # =========================================================
        Node(
            package='robot_vision',
            executable='camera_capture_node',
            name='camera_capture_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device': '/dev/video1',

                'width': 640,
                'height': 480,
                'fps': 15.0,

                'image_topic': '/image_raw',
                'status_topic': '/camera/status',

                'gst_source': 'libcamera',
                'gstreamer_pipeline': '',

                'retry_period_sec': 5.0,
                'warmup_read_attempts': 25,
                'warmup_sleep_sec': 0.05,

                'publish_status': True,

                'flip_horizontal': False,
                'flip_vertical': False,
            }]
        ),

                # Node(
        #     package='stream_ros',
        #     executable='ros_stream',
        #     name='raw_camera_stream',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[{
        #         'image_topic': '/image_raw',
        #         'host': '0.0.0.0',
        #         'port': 5000,
        #     }]
        # ),

        # =========================================================
        # 4. ARUCO DETECTOR
        # =========================================================
        Node(
            package='robot_vision',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'input_image_topic': '/image_raw',
                'processed_image_topic': '/aruco/debug_image',

                'status_topic': '/aruco/status',
                'found_topic': '/aruco/found',
                'id_topic': '/aruco/id',
                'offset_topic': '/aruco/offset',
                'width_topic': '/aruco/width',

                'fps': 8.0,

                'publish_debug_image': False,
                'publish_not_found': True,

                # Зараз у тебе fallback contour mode без cv2.aruco
                'use_cv2_aruco': False,

                'draw_full_height_frame': True,
                'frame_width_scale': 1.0,

                'contour_marker_id': 0,

                'min_contour_area': 1200.0,
                'max_contour_area': 200000.0,
                'min_square_ratio': 0.55,
                'max_square_ratio': 1.45,
                'blur_kernel': 5,
            }]
        ),

        # =========================================================
        # 5. MISSION MASTER
        # Центрування -> під'їзд до 10 см -> об'їзд -> пошук наступного
        # =========================================================
        Node(
            package='robot_vision',
            executable='mission_master_node',
            name='mission_master',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'aruco_status_topic': '/aruco/status',
                'front_distance_topic': '/lidar/front_distance',
                'cmd_topic': '/cmd_vel_raw',

                'control_hz': 20.0,

                # ---------------- ARUCO LOST GRACE ----------------

                'aruco_lost_grace_sec': 3.0,

                # Вимкнено, щоб робот не крутив 180/360 по старому offset
                'recover_turn_to_last_marker': False,
                'recover_turn_speed': 0.08,

                # Якщо мітку втратили, але ми вже близько — запускаємо об'їзд
                'assume_reached_if_lost_distance_m': 0.80,
                'assume_reached_if_lost_width_px': 80.0,

                # ---------------- FORCE BYPASS TRIGGER ----------------

                'enable_force_bypass_trigger': True,

                # Починаємо об'їзд раніше, щоб не впиратися в ArUco
                'force_bypass_if_front_below_m': 0.75,
                'force_bypass_if_width_above_px': 85.0,
                'force_bypass_if_camera_est_below_m': 0.95,

                # Захист від старої ArUco після об'їзду
                'force_reached_max_marker_age_sec': 4.0,

                # ---------------- TRACKING ----------------

                'center_deadzone_px': 90.0,
                'aruco_turn_kp': 0.0010,
                'max_track_angular': 0.12,
                'approach_speed': 0.025,

                'allow_forward_while_turning': False,
                'forward_turn_offset_limit_px': 160.0,

                # Якщо повертає не в той бік — постав True
                'invert_aruco_turn': False,

                # ---------------- STOP NEAR MARKER ----------------

                'use_lidar_for_marker_stop': True,

                # Починаємо об'їзд приблизно з 70 см
                'target_marker_distance_m': 0.70,

                # Запасний тригер по ширині мітки
                'marker_stop_width_px_override': 85.0,

                'marker_real_size_cm': 18.7,
                'camera_hfov_deg': 60.0,

                # Якщо вже ближче 45 см — точно не їдемо далі прямо
                'emergency_reached_distance_m': 0.45,

                # ---------------- SEARCH ----------------

                'search_turn_speed': 0.12,
                'search_turn_duration_sec': 0.35,
                'search_pause_sec': 0.70,

                # ---------------- CURVE BYPASS ----------------

                'marker_reached_pause_sec': 0.35,

                # -1.0 = об'їзд справа, +1.0 = об'їзд зліва
                'bypass_direction': -1.0,

                # Дуга вбік навколо ArUco
                'bypass_arc_linear_speed': 0.035,
                'bypass_arc_angular_speed': 0.32,

                # ВАЖЛИВО:
                # Дуга має кінцевий час. Не буде нескінченного кручення.
                'bypass_arc_duration_sec': 2.0,

                # Проїзд повз мітку
                'bypass_pass_forward_speed': 0.055,
                'bypass_pass_forward_duration_sec': 2.0,

                # Маленьке вирівнювання, не повний поворот назад
                'bypass_realign_angular_speed': 0.20,
                'bypass_realign_duration_sec': 0.55,

                # Якщо попереду ближче 35 см — спочатку тільки довертаємо
                'bypass_front_block_m': 0.35,

                # Максимум 0.9 сек тільки крутиться, потім іде по сценарію далі
                'bypass_turn_only_when_blocked_sec': 0.9,

                # ---------------- AFTER BYPASS ----------------

                # Після об'їзду продовжуємо рух прямо
                'after_bypass_ignore_sec': 2.5,
                'cruise_forward_speed': 0.06,
                'cruise_front_block_m': 0.26,
                'search_if_cruise_blocked': True,

                # ---------------- LIMITS / LOGS ----------------

                'max_cmd_linear': 0.20,
                'max_cmd_angular': 1.20,
                'status_log_period_sec': 2.0,
            }]
        ),
        # =========================================================
        # 6. LIDAR SAFETY
        # /cmd_vel_raw -> /cmd_vel
        #
        # ВАЖЛИВО:
        # Якщо хочеш під'їзд до мітки на 10 см,
        # stop_distance має бути менше 0.10.
        # Якщо поставити 0.25, робот зупиниться на 25 см.
        # =========================================================
        Node(
            package='robot_bringup',
            executable='lidar_safety',
            name='lidar_safety',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'scan_topic': '/scan',
                'front_distance_topic': '/lidar/front_distance',

                'input_cmd_topic': '/cmd_vel_raw',
                'output_cmd_topic': '/cmd_vel',

                # Для під'їзду до 10 см.
                # Якщо робот чіпляє мітку/стіну — підніми до 0.10 або 0.12.
                'stop_distance': 0.08,
                'warn_distance': 0.18,

                'allow_backward': True,
                'allow_rotation': True,
            }]
        ),

        # =========================================================
        # 7. MOTOR NODE
        # =========================================================
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
    ])