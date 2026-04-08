# Smart Golf Cart — 智能高尔夫球车

> ROS2 Humble | Jetson Orin NX | GPS-PID 导航 | YOLO 跟随 | v8.3.0

---

## 工程师快速上手

### 第 1 天：搭环境 + 了解系统

```
1. 搭建开发环境            → docs/DEV_GUIDE.md §2
2. 编译通过                → docs/DEV_GUIDE.md §2.1
3. 下载标准 rosbag          → 项目负责人提供的共享链接
4. 通读系统架构（必读）      → docs/ARCHITECTURE.md
```

### 第 2 天：了解你的模块

```
5. 阅读你负责的包 README     → src/你的包/README.md
   - 规控: src/golf_navigation/README.md
   - 感知: src/golf_perception/README.md
   - 地图: src/golf_mapping/README.md
6. rosbag 回放跑通你的节点   → docs/DEV_GUIDE.md §6.2
```

### 日常开发

```
7. 领取 Issue（项目负责人分配）
8. 开分支 + 写代码           → docs/DEV_GUIDE.md §3.2
9. rosbag 验证               → docs/DEV_GUIDE.md §6
10. 提交 PR（自动弹模板）     → docs/DEV_GUIDE.md §3.4
11. 等 review → 合并
```

### 需要查阅时

```
话题接口约束（改接口前必看） → docs/INTERFACE_CONTRACT.md
代码规范                    → docs/DEV_GUIDE.md §4
commit message 格式         → docs/DEV_GUIDE.md §3.3
```

---

## 文档索引

| 文档 | 内容 | 何时看 |
|------|------|--------|
| [ARCHITECTURE.md](docs/ARCHITECTURE.md) | 硬件拓扑、数据流图、状态机、节点表 | **入职必读**，通读一遍建立全局认知 |
| [DEV_GUIDE.md](docs/DEV_GUIDE.md) | 环境搭建、Git 流程、代码规范、测试方法 | **入职必读** §1-3，其余按需查阅 |
| [INTERFACE_CONTRACT.md](docs/INTERFACE_CONTRACT.md) | 28 个 ROS2 话题的 API 契约 | 改接口前必看，平时**按需查阅** |
| [TESTING_SOP.md](docs/TESTING_SOP.md) | 实车测试标准操作流程 | 项目负责人专用 |

## 包结构

| 包 | 负责人 | 说明 |
|----|--------|------|
| [golf_perception](src/golf_perception/) | 图像识别工程师 | YOLO 检测、手势识别、深度修复 |
| [golf_navigation](src/golf_navigation/) | 规控工程师 | 跟随控制、GPS-PID 导航、状态机、急停 |
| [golf_mapping](src/golf_mapping/) | 地图+传感器工程师 | GPS 路点记录、IMU 融合、测试日志 |
| [golf_communication](src/golf_communication/) | 项目负责人 | MQTT 桥接（Web APP 通信） |
| [golf_bringup](src/golf_bringup/) | 项目负责人 | Launch 文件、配置、启停脚本 |

## 快速编译

```bash
cd ~/golf_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
