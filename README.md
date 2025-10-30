# simple_person_detector

ROS 2 node wrapping OpenCV’s HOG person detector. Subscribes to an image topic, publishes annotated images, a boolean “detected” flag, and `vision_msgs/Detection2DArray` bounding boxes.

## Features
- CPU-only human detection using `cv2.HOGDescriptor`.
- Adjustable stride, padding, pyramid scale, downscale factor, and detection interval.
- Outputs:
  - `~/image_annotated` (`sensor_msgs/Image`)
  - `~/detected` (`std_msgs/Bool`)
  - `~/detection_array` (`vision_msgs/Detection2DArray`)
- Supports YAML parameter loading and `use_sim_time`.

## Build
```bash
cd ~/workspaces/robotnik_sim_ws
colcon build --packages-select simple_person_detector
source install/setup.bash
```

## Launch
```bash
ros2 launch simple_person_detector simple_person_detector.launch.py \
  config_file:=/path/to/simple_person_detector.yaml
```

## Default Parameters (`config/simple_person_detector.yaml`)
```yaml
simple_person_detector:
  ros__parameters:
    input_topic: /robot/top_ptz_rgbd_camera/color/image_raw
    win_stride: [8, 8]
    padding: [16, 16]
    scale: 1.05
    downscale_factor: 1.0
    detection_interval: 0.3
```

Override via YAML or CLI, e.g.:
```bash
ros2 run simple_person_detector detector_node --ros-args -p detection_interval:=0.5
```

## Interfaces
| Topic | Type | Notes |
|-------|------|-------|
| `<node>/image_annotated` | `sensor_msgs/Image` | Original frame + boxes |
| `<node>/detected` | `std_msgs/Bool` | True if any detection in last processed frame |
| `<node>/detection_array` | `vision_msgs/Detection2DArray` | Bounding boxes and labels |

## Dependencies
- `rclpy`, `cv_bridge`, `sensor_msgs`, `std_msgs`, `vision_msgs`
- OpenCV (system package `python3-opencv`)
