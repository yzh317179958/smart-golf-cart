# golf_perception

高尔夫球车视觉感知包 -- YOLOv8s-Pose 行人检测、手势识别、S11 相机数据修复、调试可视化。

## 节点列表

| 节点名 | 功能 | 订阅话题 | 发布话题 |
|--------|------|----------|----------|
| `detection_node` | YOLOv8s-Pose TensorRT 行人检测，输出 bbox + 17 个 COCO 关键点 + 置信度 | `/lx_camera_node/LxCamera_Rgb` (Image) | `/detections` (Detection2DArray), `/detection_keypoints` (String, JSON) |
| `gesture_node` | 基于 YOLO Pose 关键点的举手手势识别（右手=lock, 左手=stop） | `/yolo/tracking` (DetectionArray) | `/gesture_cmd` (String) |
| `camera_info_fix` | 修复 S11 相机非标准 CameraInfo 字段，使 depthimage_to_laserscan 正常工作 | `/lx_camera_node/LxCamera_TofInfo` (CameraInfo), `/lx_camera_node/LxCamera_Depth` (Image) | `/depth_camera_info_fixed` (CameraInfo), `/depth_image_fixed` (Image) |
| `image_resizer` | 将 YOLO 调试图像从原始分辨率缩小为 320x270，降低远程可视化带宽占用 | `/yolo/dbg_image` (Image) | `/dbg_small` (Image) |

## 参数说明

### detection_node

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `model_path` | string | `yolov8s-pose.engine` | TensorRT 模型文件路径 |
| `confidence_threshold` | float | 0.5 | 检测置信度阈值 |
| `image_topic` | string | `/lx_camera_node/LxCamera_Rgb` | 输入 RGB 图像话题 |
| `device` | int | 0 | GPU 设备编号 |

### gesture_node

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `threshold` | float | 15.0 | 手腕高于肩膀的像素阈值（判定举手） |
| `lock_min_frames` | int | 3 | 右手举起触发所需最小连续帧数 |
| `stop_min_frames` | int | 4 | 左手举起触发所需最小连续帧数 |
| `keypoint_confidence` | float | 0.5 | 关键点置信度阈值 |
| `hold_duration` | float | 2.0 | 手势保持时间 (秒)，超时恢复 none |
| `drop_tolerance` | int | 5 | 关键点丢帧容忍次数（防抖动） |

### camera_info_fix

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `input_topic` | string | `/lx_camera_node/LxCamera_TofInfo` | 输入 CameraInfo 话题 |
| `output_topic` | string | `/depth_camera_info_fixed` | 修复后 CameraInfo 输出话题 |
| `depth_input` | string | `/lx_camera_node/LxCamera_Depth` | 输入深度图话题 |
| `depth_output` | string | `/depth_image_fixed` | 修复后深度图输出话题 |

### image_resizer

无可配置参数。输入固定为 `/yolo/dbg_image`，输出固定为 `/dbg_small` (320x270)。

## 编译方法

```bash
cd ~/golf_ws
colcon build --packages-select golf_perception --symlink-install
source install/setup.bash
```

**额外依赖（Jetson 环境）：**

```bash
pip3 install ultralytics
# TensorRT engine 文件需预先导出：
#   yolo export model=yolov8s-pose.pt format=engine device=0
```

## 测试方法

### 单节点启动验证

```bash
# 检测节点（需要 S11 相机或 rosbag 数据）
ros2 run golf_perception detection_node --ros-args -p model_path:=/path/to/yolov8s-pose.engine
ros2 topic hz /detections

# 手势识别（需要检测节点已运行）
ros2 run golf_perception gesture_node
ros2 topic echo /gesture_cmd

# 相机信息修复
ros2 run golf_perception camera_info_fix
ros2 topic echo /depth_camera_info_fixed

# 调试图像缩放
ros2 run golf_perception image_resizer
ros2 topic hz /dbg_small
```

### rosbag 回放测试

```bash
# 回放 S11 相机数据
ros2 bag play camera_bag/ --clock
ros2 run golf_perception detection_node --ros-args -p use_sim_time:=true
ros2 run golf_perception gesture_node --ros-args -p use_sim_time:=true
```

### 检测结果验证

```bash
# 查看检测数量
ros2 topic echo /detections --field detections --no-arr

# 查看关键点 JSON（包含 bbox + 17 个 COCO 关键点坐标）
ros2 topic echo /detection_keypoints
```

## 注意事项

1. **TensorRT 模型**：`detection_node` 使用 YOLOv8s-Pose 的 TensorRT engine 文件，必须在目标 GPU (Orin NX) 上导出，不同设备的 engine 不通用。
2. **仅检测行人**：`detection_node` 只输出 COCO class 0 (person) 的检测结果，其他类别被过滤。
3. **关键点发布**：`vision_msgs/Detection2DArray` 不含关键点字段，因此 17 个 COCO 关键点通过 `/detection_keypoints` 话题以 JSON 字符串格式单独发布。
4. **S11 相机修复**：S11 输出的 CameraInfo 使用非标准畸变模型 `LX_DISTORTION_FISHEYE` 和非标准 P 矩阵，`camera_info_fix` 将其转换为标准 `plumb_bob` 模型，同时将深度图 encoding 从 `mono16` 修正为 `16UC1`。
5. **手势状态机**：`gesture_node` 输出三种状态 -- `lock`（右手举起）、`stop`（左手举起）、`none`（无手势）。带有帧数累积和保持机制，防止误触发。
6. **S11 分辨率**：RGB 为 1280x1080，深度为 240x90。`follow_target_publisher` 需要知道这两个分辨率做坐标映射。
