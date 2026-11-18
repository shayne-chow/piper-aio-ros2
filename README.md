# Piper All In One · ROS2 双臂示教与采集

本分支使用 ROS2 Humble、四路独立 CAN 和三台 Orbbec 相机采集双臂示教。操作者拖动左右主臂，从臂分别跟随。录制器将主臂位置、从臂目标、从臂反馈和三路 RGB 图像按时间戳配帧，保存为 HDF5，供后续转换为 LeRobot 训练数据。

## 环境与构建

采集主机使用 Ubuntu 22.04、ROS2 Humble 和系统 Python 3。机械臂驱动读取同级目录 `../piper_sdk` 中的 `C_PiperInterface`；其他安装位置可通过 `PIPER_SDK_PATH` 指定。本机项目目录为 `~/piper/piper-aio`。已构建的主机可直接进入下一节。

在已安装 Humble、colcon 和 Orbbec 驱动构建依赖的主机上，从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-can python3-h5py python3-numpy ros-humble-cv-bridge
(cd ros2_ws && PATH=/usr/bin:$PATH colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3)

(cd camera_ws && PATH=/usr/bin:$PATH MAKEFLAGS=-j4 colcon build \
  --build-base build_ros2 --install-base install_ros2 --packages-up-to orbbec_camera \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3)
```

## 启动与录制

以下四个终端均从仓库根目录运行。

### 1. 启动三台相机

以下为采集主机已核对的序列号。更换设备时按实际安装位置更新，保持前置、左腕、右腕的顺序与训练配置一致。

```bash
./start_cam_ros2.sh camera_launch:=gemini_330_series.launch.py \
  front_serial:=CP2C8630017A left_serial:=CP7JC42000B4 right_serial:=CP7JC42000M1
```

默认采集 640 × 480、30 fps 彩色图像。录制器订阅三个命名空间 `/camera_f`、`/camera_l`、`/camera_r` 下的 `color/image_raw/compressed`，配帧后解码为 RGB8。若需深度，在相机 launch 添加 `enable_depth:=true`，并在录制器添加 `-p use_depth:=true`。

### 2. 启动主从臂

首次配置或 USB 拔插后，用 `bash piper_ws/scripts/find_all_can_port.sh` 核对物理端口；确认映射后运行 `bash piper_ws/scripts/can_config_dagger.sh`。四个接口应为 `can_left_lea`、`can_right_lea`、`can_left_fol`、`can_right_fol`。

**默认启动会自动进入拖动示教，从臂可能直接向主臂当前位置运动。** 先对齐左右每组主从臂，清空运动范围并保持急停可及。仅查看反馈时使用 `./start_ros2.sh auto_teach:=false`。同一从臂 CAN 同时只运行一个控制驱动。

```bash
./start_ros2.sh
```

### 3. 启动录制器

```bash
./collect_ros2.sh --ros-args \
  -p dataset_dir:="$HOME/piper/data" -p task_name:=demo_ros2
```

### 4. 开始和保存一段示教

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /teleop/status --once
ros2 service call /record/start std_srvs/srv/Trigger '{}'
```

确认跟随状态为 `ACTIVE` 后执行示教。录制器仅在所需数据流都有效且时间差满足条件时写帧。动作完成后保存：

```bash
ros2 service call /record/save std_srvs/srv/Trigger '{}'
```

保存响应给出完整帧数和文件路径，如 `~/piper/data/demo_ros2/episode_0.hdf5`。下一段示教重复 start/save 即可。放弃当前录制使用 `/record/discard`。结束跟随使用 `ros2 service call /teleop/stop std_srvs/srv/Trigger '{}'`；主臂退出拖动模式使用 `ros2 service call /teleop/teach std_srvs/srv/SetBool '{data: false}'`。

## 数据格式与训练含义

关节向量为 14 维：左臂 J1–J6、左夹爪、右臂 J1–J6、右夹爪。关节角使用 rad，夹爪开度使用 m。

| HDF5 字段                                                         | 物理含义                               | LeRobot 转换                                               |
| ----------------------------------------------------------------- | -------------------------------------- | ---------------------------------------------------------- |
| `observations/qpos`                                             | 从臂实际关节位置和夹爪开度             | `observation.state`                                      |
| `action_applied`                                                | 驱动按固件位置边界裁剪后发给从臂的目标 | 优先用作 `action`                                        |
| `action`                                                        | 主臂原始目标，可能超出从臂可执行范围   | 缺少 `action_applied` 的旧文件回退到此字段               |
| `observations/images/{cam_high,cam_left_wrist,cam_right_wrist}` | 三路解码后的 RGB8 图像                 | 对应三路 `observation.images.*`                          |
| `observations/eef_pose`                                         | 左右从臂末端位姿与夹爪                 | 当前帧为 `observation.eef_pose`，下一帧为 `eef_action` |
| `timestamps/*`                                                  | 样本时间及各路配帧时间                 | 用于检查采样间隔和动作与视频的时间差                       |

文件还包含 `observations/qvel`、`observations/effort` 和逐帧 `collect` 标签。采集目标为 30 Hz，实际节奏以 `timestamps/frame` 为准。末端位姿消息使用接收时间，关节和图像使用消息头时间。

本机动态示教样本 `episode_4` 共 1,952 帧，跨度 66.28 s，有效采样率 29.44 Hz。逐帧比较三路图像与左右从臂位置反馈，最大时间戳差为 22.64 ms。这些数值是该次记录的实测结果。

## 转换为 LeRobot

在独立的 Python 3.10 环境中安装仓库固定的 LeRobot 版本和 FFmpeg。转换前准备一个只含所选完整 episode 的输入目录；转换器会扫描其中所有 `*.hdf5`，包括扩展名匹配的临时文件。

```bash
git submodule update --init lerobot
conda create -n lerobot python=3.10 -y
conda activate lerobot
conda install -c conda-forge ffmpeg -y
python -m pip install -e ./lerobot
python -m pip install 'datasets==3.6.0' h5py
```

下面仅转换动态样本 `episode_4`。示例使用新的输出目录，因为转换器会删除已有的同名目标数据集。转换器目前用 `INSTRUCTION = "Example"` 写入任务文本；用于语言条件策略微调时，应先将它设为该批示教的实际任务指令。

```bash
stage=$(mktemp -d)
mkdir "$stage/demo_ros2"
ln -s "$HOME/piper/data/demo_ros2/episode_4.hdf5" "$stage/demo_ros2/episode_4.hdf5"
output=$(mktemp -d "$HOME/piper/data/lerobot-export-XXXXXX")
python convert_data/convert_to_lerobot.py \
  --src_path "$stage" --repo_ids demo_ros2 \
  --tgt_path "$output" --save_repoid piper_demo
```

转换器用下一帧构造末端动作，因此在默认裁剪设置下，1,952 帧对应 1,951 个训练样本。本分支已核验该样本的观测和动作映射；完整数据集导出与训练加载应在所用训练环境中验收。

## π0.5 真机推理

部署微调后的 π0.5 时，可复用本仓库的相机配置，在 GPU 主机运行 openpi 策略服务，在机器人端使用 AgileX 官方 ROS2 驱动控制左右从臂。两端之间的 ROS2 推理客户端仍需实现：按 checkpoint 的训练配置组装三路 RGB、从臂状态和任务指令，再按训练时的单位、左右臂顺序及时间步长执行预测动作块。采集时的四 CAN 驱动与推理时的官方驱动分开运行。

- [openpi 策略服务与客户端](https://github.com/Physical-Intelligence/openpi/blob/main/docs/remote_inference.md)
- [AgileX 官方 ROS2 驱动](https://github.com/agilexrobotics/agx_arm_ros)

## 项目来源

本项目沿用 Piper All In One 的示教与数据转换工作，并使用 AgileX PiPER SDK、Orbbec ROS2 驱动和 LeRobot。许可证见 [LICENSE](LICENSE)。
