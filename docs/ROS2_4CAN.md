# ROS2 Humble 四 CAN 示教采集

ROS1 原入口保留。新实现位于 `ros2_ws/src`，不依赖 ROS1 bridge。
目标是人工拖动左右主臂，主机根据主臂反馈生成限速目标，左右从臂分别跟随。

## 环境与构建

使用 Ubuntu 22.04 的 `/usr/bin/python3`，避免 Conda Python 与系统 rclpy/cv_bridge ABI 混用。
本机 SDK 在 `/home/top/piper/piper_sdk`，启动脚本通过 PYTHONPATH 引用它；可用 PIPER_SDK_PATH 覆盖。
当前实现使用 SDK 的 legacy/default `C_PiperInterface`。更换固件后必须重新核对协议版本。

```bash
cd /home/top/piper/piper-aio
source /opt/ros/humble/setup.bash
/usr/bin/python3 -m pip install --user h5py
cd ros2_ws
PATH=/usr/bin:$PATH /usr/bin/colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

## CAN 映射与只读启动

| USB        | 角色   | 接口          |
| ---------- | ------ | ------------- |
| 3-12.2:1.0 | 左从臂 | can_left_fol  |
| 3-12.1:1.0 | 右从臂 | can_right_fol |
| 3-11.4:1.0 | 左主臂 | can_left_lea  |
| 3-11.2:1.0 | 右主臂 | can_right_lea |

```bash
cd /home/top/piper/piper-aio
bash piper_ws/scripts/can_config_dagger.sh
./start_ros2.sh
```

USB 拔插会恢复成 can0/can1 且 DOWN，必须重新执行映射脚本。
默认启动只接收 CAN，不使能、不回零、不改变机械臂模式。每路使用进程锁防止本驱动重复启动；不要同时运行其他 CAN 控制程序。

第二个终端：

```bash
source /opt/ros/humble/setup.bash
source /home/top/piper/piper-aio/ros2_ws/install/setup.bash
ros2 topic echo /teleop/status
```

## 主臂反馈与拖动

四 CAN 反馈方案使用 `leader_source=feedback`（默认）。主臂的通信角色为 0xFC，拖动示教是单独的工作模式。
0xFA 是原生主从总线示教输入角色，不保证发布 0x2A5..0x2A8 反馈。
本机实测：两只主臂原先只能回复固件查询；设置 0xFC 后开始持续发布约 200 Hz 状态。

`./start_ros2.sh` 看到主臂状态和关节反馈后，会自动完成原来的三步：主臂切到反馈角色、进入拖动示教，并从臂开始跟踪。日志出现 `Drag-teach on. Followers track leader joints.` 后即可拖主臂。从臂会直接走到主臂当前关节位置。
专家兼容选项 `./start_ros2.sh leader_source:=control` 改读 0x155/156/157/159，不会凭“关节值为零”自动切换来源。默认反馈方案才是本机验证路径。

## 启动、停止跟随

启动脚本默认 `auto_teach:=true`。跟随开始后不再检查对齐角度、软件限位或跟踪误差，也不做限速插值。固件不接收 J2 < 0°、J3 > 0° 和负夹爪，发出去的目标只把这三项夹到边界。

`/teleop/stop` 之后不会再次自动进入示教。需要恢复时重新运行 `./start_ros2.sh`，或手动调用：

```bash
ros2 service call /teleop/prepare_leaders std_srvs/srv/Trigger '{}'
ros2 service call /teleop/teach std_srvs/srv/SetBool '{data: true}'
ros2 service call /teleop/start std_srvs/srv/Trigger '{}'
ros2 topic echo /teleop/status
```

默认 100 Hz、速度比例 100%。
进程正常退出也停止活动中的从臂，但进程被 SIGKILL、主机掉电或 CAN 物理断线时不能保证发出停止帧，软件不能代替硬件急停。
停止跟随后主臂仍可保持拖动模式；可用 `teach {data: false}` 显式退出。

通过 `ros2 run piper_aio_ros2 four_can --ros-args -p ...` 设置启动参数。参数运行中只读，包括
`rate/speed_percent/angular_speed/gripper_speed/start_angle/start_gripper/max_tracking_error/lower_limits/upper_limits`。
关节限位：[-150,150]、[-10,180]、[-170,10]、[-100,100]、[-70,70]、[-180,180] 度；夹爪 [-0.002,0.08] 米。
J2 下限和 J3 上限相对固件查询的 0° 各放宽 10°，用来容纳未上电时零位附近的下坠；其余行程限位仍在。

## 相机

新的相机构建独立于原有不完整 install，保留已有 Orbbec 源码修改：

```bash
cd /home/top/piper/piper-aio/camera_ws
source /opt/ros/humble/setup.bash
PATH=/usr/bin:$PATH MAKEFLAGS=-j4 /usr/bin/colcon build \
  --build-base build_ros2 --install-base install_ros2 --packages-up-to orbbec_camera \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

`start_cam_ros2.sh` 要求实际相机型号对应的 launch 文件名及三台序列号，不猜测设备位置：

```bash
./start_cam_ros2.sh camera_launch:=gemini_330_series.launch.py \
  front_serial:=实际前置序列号 left_serial:=实际左腕序列号 right_serial:=实际右腕序列号
```

这是 Gemini 330 系列的调用示例；其他型号传入对应 Orbbec launch 文件。
默认名称 camera_f/camera_l/camera_r，彩色图像 640x480@30；深度默认关闭。
本次检查 USB 列表未发现相机，未验证真实图像。不同型号/版本的话题以 `ros2 topic list` 为准，可在采集节点覆盖 image_topics/depth_topics。

## 采集

```bash
./collect_ros2.sh --ros-args -p dataset_dir:=/home/top/data -p task_name:=demo
# 另一个已 source ROS2 环境的终端
ros2 service call /record/start std_srvs/srv/Trigger '{}'
ros2 service call /record/save std_srvs/srv/Trigger '{}'
# 丢弃当前 episode：
ros2 service call /record/discard std_srvs/srv/Trigger '{}'
```

仅在跟随 ACTIVE 且全部数据源有效时写帧。默认最大数据年龄 0.2 秒，最大帧内时间差 0.05 秒。
图像订阅使用兼容传感器发布端的 best-effort QoS；有界队列、逐帧写盘，不把整段图像放进内存。
PosCmd 没有 header，末端姿态流使用 ROS 接收时间；关节/图像使用消息时间，无法声称硬件级同步。

保留原格式：`observations/qpos/qvel/effort/eef_pose`（14维）、三路 `observations/images/*`、`action`（14维）、`collect`。
`action` 是主臂绝对目标，新增 `action_applied` 为限速后实际发送的从臂目标；新增 `timestamps/*` 和格式属性。
图像明确转换为 RGB8，尺寸采用实际图像尺寸；默认相机名与原 LeRobot 转换器一致。
采集不会再调用 ROS1 两 CAN 的回零/恢复模式服务；ROS1 推理与 replay 脚本本次未迁移。
未保存/异常中断的文件保留为 `.partial.hdf5`；只有显式 save 才发布成 `episode_N.hdf5`，不会覆盖已有文件。

## 验证

```bash
source /opt/ros/humble/setup.bash
source /home/top/piper/piper-aio/ros2_ws/install/setup.bash
export PYTHONPATH=/home/top/piper/piper_sdk:$PYTHONPATH
cd /home/top/piper/piper-aio
/usr/bin/python3 -m unittest discover -s ros2_ws/src/piper_aio_ros2/test -v
```

测试覆盖非法目标、合法零位、数据过期/缺帧、同步窗口、限速、只读启动、拒绝不完整数据、故障双臂停止且不自动恢复、主臂反馈角色和 HDF5 回读。
真实 CAN 验证已读取四臂状态；只读运行时四路 TX 增量均为 0。
未完成真实主从运动验证：当前姿态/夹爪未对齐且部分关节位于默认限位之外。没有自动运动绕过这些检查。

补充验证：隔离 ROS_DOMAIN_ID=197 的真实 DDS 发布/订阅测试已通过，包括 reliable 关节消息与 best-effort 图像到 HDF5 的端到端采集。复现：

```bash
ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 ros2_ws/src/piper_aio_ros2/test/dds_smoke.py
```

构建和测试结果保存在 `docs/validation/`。人工对齐完成前只读运行，不启动跟随。
