# Synthetic Camera and Detector Setup

A small ROS 2 pipeline that detects a coloured cube in a top-down camera image, converts the detection into a 3D pose in the robot's base frame, and sends a UR5e arm to a pre-grasp pose above it. Everything runs on mock hardware and a synthetic camera, so it works on a laptop without a GPU or a physical robot.

## Goal

Demonstrate the full chain from pixels to robot motion:

```
image -> detect -> pose in robot frame -> plan -> move
```

The cube position is set by the user, so it serves as ground truth for measuring the detector. 

| Node | Package | Role |
|---|---|---|
| `synthetic_camera` | `cube_perception` | Renders a coloured square (top view of a cube) from known ground-truth position; publishes `Image` and `CameraInfo` |
| `static_transform_publisher` | `tf2_ros` | Publishes `base_link -> camera_optical_frame` (camera above the table, optical axis pointing down) |
| `object_detector` | `cube_perception` | Segments the cube, back-projects the centroid, transforms it into `base_link`, publishes `/object_pose` |
| `planner client` | `ur_planner_client` | Subscribes to `/object_pose`, adds a pre-grasp offset, plans and executes |

### How the detector works

1. Convert the ROS image to OpenCV BGR (`cv_bridge`), then to HSV and threshold with `cv2.inRange`.
2. Morphological opening removes speckle.
3. Take the largest contour, reject it if its area is below `min_area_px`, and compute a sub-pixel centroid `(u, v)` from image moments.
4. Back-project with the pinhole model: `X = (u - cx) * Z / fx`, `Y = (v - cy) * Z / fy`, where `Z = camera_height - object_top_z`.
5. Transform the point from the camera frame to `base_link` with TF2 and publish it as `PoseStamped` with the image timestamp, so detections can be matched to ground truth by time.


## What I learned and debugged

- The detector crashed with `"base_link" passed to lookupTransform argument target_frame does not exist`. `ros2 topic echo /tf_static` showed that nothing was publishing the camera transform at all. The code was fine; the transform publisher had not been started. Fix: start it from the launch file, and wrap TF lookups in `try/except tf2_ros.TransformException` so a missing transform produces a warning, not a crash.

- The detector's `object_top_z` was at its default (0.05) while the true value was 0.10, and x/y were off by about 11 mm and 14 mm. Lesson: z is an *input* in this method, not a measurement, and the same number must be passed to every node through one source of truth.

- `lookup_transform` fetches, `buffer.transform` applies. The first returns a `TransformStamped` and takes a time; the second converts a stamped message and takes the time from its header.


## Limitations

- No real sensor noise, lens distortion, lighting variation or calibration error
- Depth equals camera height minus `object_top_z`, which is valid only when the optical axis is perpendicular to the table. A tilted camera needs a ray-plane intersection instead.
- Real perspective shows the side faces slightly and biases the centroid off-axis.
- z is not measured; it comes from a parameter.
- The detector publishes an identity quaternion; the planner builds the gripper orientation itself.
