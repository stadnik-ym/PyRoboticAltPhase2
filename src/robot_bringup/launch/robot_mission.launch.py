from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ============================================================
        # LD06 LIDAR NODE
        # ============================================================
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

                'angle_resolution_deg': 1.0,
                'angle_offset_deg': 0.0,
                'invert_angle_direction': False,

                # Передній сектор для швидкої оцінки перешкоди
                'front_min_deg': -45.0,
                'front_max_deg': 45.0,

                # Важливо для зменшення затримки по фронтальній дистанції
                'immediate_front_publish': True,

                # Для діагностики можна лишити 0
                'min_confidence': 0,
            }]
        ),

        # ============================================================
        # CAMERA CAPTURE NODE
        # ============================================================
        Node(
            package='camera',
            executable='camera_capture_node',
            name='camera_capture_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # Якщо камера у тебе не /dev/video0 — заміни тут
                'device': '/dev/video0',

                'width': 640,
                'height': 480,
                'fps': 15.0,

                'image_topic': '/image_raw',

                # Головний фікс проти:
                # Corrupt JPEG data: ... extraneous bytes before marker
                #
                # YUYV не використовує JPEG-декодування, тому ці повідомлення
                # зазвичай зникають повністю.
                'fourcc': 'YUYV',

                # Якщо камера фізично все одно примусово працює через MJPG,
                # ці системні libjpeg-warning будуть приглушені.
                'suppress_jpeg_warnings': True,

                # Мінімальна черга кадрів, щоб не було великої затримки
                'buffer_size': 1,

                # Якщо кадри не приходять, камера перевідкриється
                'reopen_after_failures': 30,
            }]
        ),

        # ============================================================
        # VISION PROCESSOR / ARUCO DETECTOR NODE
        # ============================================================
        Node(
            package='camera',
            executable='vision_processor_node',
            name='vision_processor_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'input_image_topic': '/image_raw',
                'processed_image_topic': '/image_processed',

                # ВАЖЛИВО:
                # Саме ці топіки має слухати mission_master.
                'aruco_id_topic': '/aruco/id',
                'aruco_offset_topic': '/aruco/offset',
                'aruco_width_topic': '/aruco/width',

                # Якщо твої маркери створені як DICT_ARUCO_ORIGINAL — лишай так.
                # Якщо генерував 4x4 — заміни на DICT_4X4_50.
                'aruco_dictionary': 'DICT_ARUCO_ORIGINAL',

                # Публікувати картинку з рамкою маркера
                'publish_processed': True,

                # Покращення контрасту для ArUco
                'use_clahe': True,

                # Різкість. Краще спочатку False.
                # Якщо маркер мильний — можна поставити True.
                'use_sharpen': False,

                'debug_log_period_sec': 1.0,
            }]
        ),

        # ============================================================
        # ROS IMAGE STREAM TO BROWSER
        # ============================================================
        Node(
            package='stream_ros',
            executable='ros_stream',
            name='ros_stream',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # Краще стрімити processed, щоб бачити рамку ArUco
                'image_topic': '/image_processed',

                # Якщо хочеш бачити чисту камеру — заміни на /image_raw
                # 'image_topic': '/image_raw',

                'host': '0.0.0.0',
                'port': 5000,
                'jpeg_quality': 80,
            }]
        ),

        # ============================================================
        # MISSION MASTER
        # ============================================================
        Node(
            package='robot_bringup',
            executable='mission_master',
            name='mission_master',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # Mission не має йти напряму в мотори.
                # Вона дає команду в raw, а lidar_safety вже фільтрує.
                'cmd_topic': '/cmd_vel_raw',

                # ArUco-вхід з vision_processor_node
                'aruco_id_topic': '/aruco/id',
                'aruco_offset_topic': '/aruco/offset',
                'aruco_width_topic': '/aruco/width',

                # LiDAR-вхід
                'front_distance_topic': '/lidar/front_distance',
                'closest_topic': '/lidar/closest',

                # ----------------------------------------------------
                # ARUCO TRACKING
                # ----------------------------------------------------
                'aruco_lost_grace_sec': 1.2,

                # Коли marker width більший за це значення — робот вважає,
                # що під'їхав достатньо близько.
                'marker_stop_width': 420.0,

                # Мертва зона по центру кадру
                'center_deadzone_px': 80.0,

                # Коефіцієнт повороту на маркер
                'aruco_turn_kp': 0.0025,

                # Максимальна кутова швидкість при трекінгу
                'max_track_angular': 0.30,

                # Швидкість підʼїзду до маркера
                'approach_speed': 0.05,

                # Дозволити їхати вперед, коли робот ще трохи довертає
                'allow_forward_while_turning': True,

                # Якщо offset більший за це — краще не їхати вперед,
                # а спочатку довернутися.
                'forward_turn_offset_limit_px': 320.0,

                # ----------------------------------------------------
                # SECTOR SEARCH
                # ----------------------------------------------------
                # Поворот до наступного сектору приблизно на 20 градусів
                'sector_turn_angle_deg': 20.0,

                # Швидкість повороту між секторами
                'sector_turn_speed': 0.8,

                # Час зупинки для аналізу сектору
                'sector_analyze_duration_sec': 0.35,

                # Максимальна кількість секторів перед повторним циклом
                'max_search_sectors': 18,

                # ----------------------------------------------------
                # OBSTACLE AVOIDANCE
                # ----------------------------------------------------
                'enable_avoidance': True,

                # -1.0 — обʼїзд в одну сторону, 1.0 — в іншу
                'avoid_direction': -1.0,

                'avoid_turn_speed': 0.7,
                'avoid_turn_duration_sec': 1.1,

                'avoid_forward_speed': 0.07,
                'avoid_forward_duration_sec': 1.3,

                'avoid_return_turn_duration_sec': 1.1,
                'avoid_pass_forward_duration_sec': 0.9,
            }]
        ),

        # ============================================================
        # LIDAR SAFETY FILTER
        # ============================================================
        Node(
            package='robot_bringup',
            executable='lidar_safety',
            name='lidar_safety',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'scan_topic': '/scan',

                # Вхід від mission_master
                'input_cmd_topic': '/cmd_vel_raw',

                # Вихід у моторну ноду
                'output_cmd_topic': '/cmd_vel',

                # Безпечна дистанція.
                # Можеш підняти до 0.45, якщо робот пізно гальмує.
                'stop_dist': 0.40,

                # Передній кут контролю
                'front_angle_deg': 45.0,

                # Дозволити назад, коли попереду перешкода
                'allow_backward': True,

                # Якщо True — safety сама надсилає 0, коли бачить перешкоду
                'auto_stop': True,

                # Як часто писати лог
                'debug_period_sec': 0.5,
            }]
        ),

        # ============================================================
        # DIFFERENTIAL DRIVE MOTOR NODE
        # ============================================================
        Node(
            package='diff_drive_l298n',
            executable='diff_drive_node',
            name='diff_drive_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # Моторна нода слухає вже безпечну команду
                'cmd_topic': '/cmd_vel',

                # GPIO pins
                'ENA': 17,
                'IN1': 27,
                'IN2': 22,

                'ENB': 13,
                'IN3': 26,
                'IN4': 19,

                'pwm_freq': 700,

                # Якщо мотори пищать і не рушають — піднімай.
                # У тебе раніше 70-80 вже було майже на межі,
                # тому стартово лишаємо 80.
                'min_pwm': 80.0,

                'max_linear': 0.25,
                'max_angular': 2.0,

                'wheel_base': 0.16,

                # Якщо команда не приходить — стоп
                'cmd_timeout': 0.5,
            }]
        ),
    ])