from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # TODO 1: camera position defined ONCE. Both the TF publisher and the
    # synthetic camera read from this dict, so they cannot drift apart.
    cam = {'cam_x': 0.50, 'cam_y': 0.0, 'cam_z': 0.80}

    # TODO 2: static transform base_link -> camera_optical_frame.
    # Quaternion (qx, qy, qz, qw) = (1, 0, 0, 0) is a 180 degree rotation about x:
    #   x_opt = +x_base, y_opt = -y_base, z_opt = -z_base.
    # The optical axis (z_opt) therefore points straight down, matching Z_opt = -Z_base.
    # All arguments must be strings, hence str(...).
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            '--x', str(cam['cam_x']),
            '--y', str(cam['cam_y']),
            '--z', str(cam['cam_z']),
            '--qx', '1.0', '--qy', '0.0', '--qz', '0.0', '--qw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_optical_frame',
        ],
        output='screen',
    )

    # TODO 3: synthetic camera. The dict goes in directly as parameters.
    # The node must declare cam_x, cam_y, cam_z (declare_parameter) to receive them.
    synthetic_camera = Node(
        package='cube_perception',
        executable='synthetic_camera',   # <- name from setup.py entry_points
        name='synthetic_camera',
        parameters=[cam],
        output='screen',
    )

    # TODO 4: detector. The node name must match super().__init__('object_detector')
    # so that parameters and `ros2 param set /object_detector ...` apply.
    detector = Node(
        package='cube_perception',
        executable='detector',           # <- name from setup.py entry_points
        name='object_detector',
        output='screen',
    )

    return LaunchDescription([
        # TODO 5
        static_tf,
        synthetic_camera,
        detector,
    ])