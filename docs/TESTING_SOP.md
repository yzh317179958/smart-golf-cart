# 实车测试标准操作流程 (SOP)

> 适用角色：项目负责人（实车测试）
> 更新日期：2026-04-08

---

## 1. 测试前准备

### 1.1 合并代码并同步

```bash
# VPS 上拉取最新代码
cd ~/projects/golf_ws
git pull origin main

# 同步到 WheelTec
rsync -avz --delete \
  --exclude='build/' --exclude='install/' --exclude='log/' \
  --exclude='.git/' --exclude='__pycache__/' --exclude='*.pyc' \
  --exclude='*.engine' --exclude='third_party/lx_camera_ros/' \
  -e "ssh -p 2225" \
  src/ wheeltec@localhost:~/golf_ws/src/
```

### 1.2 WheelTec 编译

```bash
ssh wheeltec-4g
cd ~/golf_ws
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
colcon build --packages-select golf_navigation golf_mapping golf_communication golf_perception golf_bringup
```

> 编译失败 → 不要实车测试，开 Issue 退回给对应工程师。

### 1.3 硬件检查

| 检查项 | 命令/方法 | 通过标准 |
|--------|----------|---------|
| 电池电压 | 底盘显示屏 | ≥ 22V |
| GPS 天线 | 目视确认 | ANT1(前) ANT2(后) 未松动 |
| LiDAR | 目视确认 | 未被遮挡 |
| S11 网线 | 目视确认 | 网线插紧（松动会 LX_E_RECONNECTING） |
| 4G 信号 | `ssh wheeltec-4g` 能连上 | 隧道正常 |
| 手柄 | 开机，按键测试 | 急停按钮(0)、恢复按钮(1) 响应 |

---

## 2. 启动系统

```bash
# 1. 清理旧进程（铁律）
sudo pkill -9 -f ros2

# 2. 启动全栈
bash ~/golf_ws/start_all.sh

# 3. 等待 30 秒后验证关键话题
ros2 topic hz /scan              # 应 ~12Hz
ros2 topic hz /gps/fix           # 应 1-5Hz
ros2 topic hz /system_mode       # 应 10Hz
ros2 topic hz /yolo/tracking     # 应 ~15Hz（需有人在镜头前）
```

> 任何话题无数据 → 不要开始测试，先排查。

---

## 3. 标准测试项目

### 3.1 跟随测试（2-3 分钟）

**开始录制：**
```bash
# 新 tmux 窗口
bash ~/golf_ws/scripts/record_follow.sh
```

**测试动作（按顺序）：**

| 序号 | 动作 | 预期结果 | 观察点 |
|:---:|------|---------|--------|
| 1 | 站在车前 1.5m 处 | /follow_state = tracking | 锁定成功 |
| 2 | 直线前走 10m | 车跟随，保持 ~1m 距离 | 平稳无顿挫 |
| 3 | 左转 90° 继续走 | 车转弯跟随 | 转弯流畅 |
| 4 | 停下不动 3s | 车停下 | 不前冲 |
| 5 | 快速侧移离开视野 | /follow_state = idle，车停 | 丢人后停车 |
| 6 | 回到车前 | 重新锁定跟随 | 自动恢复 |
| 7 | 举左手 2s | 暂停跟随 | 手势识别 |

**结束：** Ctrl+C 停止录制。

### 3.2 导航测试（3-5 分钟）

**开始录制：**
```bash
bash ~/golf_ws/scripts/record_nav.sh
```

**测试动作：**

| 序号 | 动作 | 预期结果 |
|:---:|------|---------|
| 1 | 手机打开 APP，点一个近距离路点 → Navigate Here | 车开始导航 |
| 2 | 观察行驶过程 | 直行稳定、弯道温柔修正 |
| 3 | 观察到达 | APP 显示 arrived，车自动切回 FOLLOWING |
| 4 | 再测一个远距离路点 | 多段路径通过 |
| 5 | 导航途中按手柄急停键 | 车立即停止，APP 显示 E_STOP |
| 6 | 按手柄恢复键 | 切回 FOLLOWING |

**结束：** Ctrl+C 停止录制。

### 3.3 安全测试

| 场景 | 操作 | 预期 |
|------|------|------|
| LiDAR 急停 | 导航中，人站在车前 2m 内 | 车减速/停车 |
| 手柄急停 | 任何模式下按按钮 0 | 立即停车 |
| 手柄恢复 | E_STOP 状态按按钮 1 | 切回 FOLLOWING |
| APP 急停 | APP 点 STOP 按钮 | 立即停车 |
| APP 恢复 | APP 点 RESUME 按钮 | 切回 FOLLOWING |

---

## 4. 发现 Bug 时

### 4.1 现场处理

```bash
# 1. 手柄急停保证安全
# 2. 立刻录一段 bug 复现 bag（如果还没在录的话）
bash ~/golf_ws/scripts/record_follow.sh   # 或 record_nav.sh
# 3. 复现 bug 操作
# 4. Ctrl+C 停止录制
```

### 4.2 收集证据

```bash
# 截取最近日志（tmux 各窗口）
tmux capture-pane -t golf:0 -p > /tmp/sensor_log.txt
tmux capture-pane -t golf:1 -p > /tmp/perception_log.txt
tmux capture-pane -t golf:2 -p > /tmp/follow_log.txt

# 拷贝到 VPS
scp -P 2225 /tmp/*_log.txt wheeltec@localhost:/tmp/
# 或从 VPS 拉
scp -P 2225 wheeltec@localhost:/tmp/*_log.txt /root/projects/golf_ws/data/
```

### 4.3 开 GitHub Issue

标题格式：`[bug][包名] 简述现象`

内容模板：
```
## 现象
在 XX 测试中，执行 YY 操作时，车辆 ZZ。

## 复现步骤
1. 启动 start_all.sh
2. ...
3. ...

## 预期行为
...

## 实际行为
...

## 证据
- rosbag: [共享链接]
- 日志: 见附件
- 版本: v8.3.x (commit: xxxxxx)

## 影响模块
golf_navigation / golf_perception / golf_mapping / golf_communication
```

---

## 5. 测试通过

```bash
# 1. 停止系统
bash ~/golf_ws/stop_all.sh

# 2. 验证 bag 完整性
bash ~/golf_ws/scripts/verify_bag.sh ~/golf_ws/bags/follow_bag_xxx
bash ~/golf_ws/scripts/verify_bag.sh ~/golf_ws/bags/nav_bag_xxx

# 3. scp bag 回 VPS
scp -P 2225 wheeltec@localhost:~/golf_ws/bags/*.db3 /root/projects/golf_ws/data/bags/

# 4. 打版本 tag
cd ~/projects/golf_ws
git tag v8.3.x
git push origin main --tags

# 5. 上传 bag 到共享存储（网盘），更新下载链接通知工程师
```

---

## 6. 测试频率

| 类型 | 频率 | 触发条件 |
|------|------|---------|
| 远程编译验证 | 每周 1-2 次 | 有 PR 合并时 |
| 实车集成测试 | 每 1-2 周 | 攒够一批改动 |
| 紧急测试 | 随时 | 关键 bug 修复后 |

---

## 快速参考

```bash
# 一键启动
bash ~/golf_ws/start_all.sh

# 一键停止
bash ~/golf_ws/stop_all.sh

# 录跟随 bag
bash ~/golf_ws/scripts/record_follow.sh

# 录导航 bag
bash ~/golf_ws/scripts/record_nav.sh

# 验证 bag
bash ~/golf_ws/scripts/verify_bag.sh ~/golf_ws/bags/xxx

# 查看 tmux 日志
tmux attach -t golf
```
