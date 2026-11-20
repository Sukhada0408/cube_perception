from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # define camera position
    cam = {'cam_x': 0.50, 'cam_y': 0.0, 'cam_z': 0.80}

    # static transform base_link -> camera_optical_frame.
    # quaternion (qx, qy, qz, qw) = (1, 0, 0, 0) is a 180 degree rotation about x
    # x_opt = +x_base, y_opt = -y_base, z_opt = -z_base.
    # Z_opt = -Z_base.
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

    # synthetic camera node
    synthetic_camera = Node(
        package='cube_perception',
        executable='synthetic_camera',   # <- name from setup.py entry_points
        name='synthetic_camera',
        parameters=[cam],
        output='screen',
    )

    # detector node
    detector = Node(
        package='cube_perception',
        executable='detector',
        name='object_detector',
        output='screen',
    )

    return LaunchDescription([
        static_tf,
        synthetic_camera,
        detector,
    ])