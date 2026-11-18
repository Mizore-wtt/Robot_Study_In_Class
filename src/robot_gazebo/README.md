# 实验 9：Scout 机器人 ROS 2 仿真

本目录基于课程“实验9相关文件”中的 `robot_gazebo.zip`、`gazebo_world.zip`、
`gazebo_apriltag_models.zip` 和 `gazebo_models.zip` 改造。
运行环境为 **Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11**。

当前阶段完成机器人模型、四轮差速运动、16 线激光雷达、RGB-D 相机、IMU、轮式里程计、
二维扫描预处理、TF 和可视化、AprilTag 检测，以及指导书第 6 节的 Cartographer
建图、landmark 融合、地图导出和加载已有地图进行纯定位。

## 1. 目录规则与包的职责

继续使用已有的 `src/robot_gazebo`，不再嵌套另一个工作空间。
工作空间根目录是 `/home/ubuntu22/study/slam`。

```text
slam/
├── src/robot_gazebo/
│   ├── scout_gazebo/                    # 主仿真包，ament_cmake
│   │   ├── package.xml                  # ROS 2 包元数据与运行依赖
│   │   ├── CMakeLists.txt               # 安装资源和可执行脚本
│   │   ├── launch/scout_gazebo.launch.py # 统一启动入口
│   │   ├── urdf/                       # 底盘、轮子、支架、IMU、运动插件
│   │   ├── meshes/                     # 课程提供的 Scout 外观
│   │   ├── models/                     # 停车场依赖的车辆和 AprilTag 资源
│   │   ├── worlds/                     # 课程停车场及原来的 lab/office 场景
│   │   ├── config/                     # 传感器参数、RViz2 配置
│   │   └── scripts/                    # 点云转扫描、仿真验收工具
│   ├── realsense_ros_gazebo/            # D435 几何 + ROS 2 标准 RGB-D 插件
│   ├── apriltag/                        # 课程 apriltag.zip 中的 3.4.2 原生库
│   ├── apriltag_ros/                    # ROS 2 检测适配、课程消息、参数、验收工具
│   ├── velodyne_simulator/
│   │   └── velodyne_description/        # VLP-16 几何 + ROS 2 标准雷达插件
│   └── 其他原始 ROS 1 包/               # 保留源码，COLCON_IGNORE 排除编译
├── build/                              # colcon 自动生成
├── install/                            # colcon 自动生成
├── log/                                # 构建/启动日志与验收报告
└── 实验9相关文件/                        # 原始课程资源，保持不变
```

`models/` 是 Gazebo 模型资源目录，`scripts/` 是 ROS 包中的可执行脚本目录。
它们都属于现有的 `scout_gazebo` 包。只提取停车场实际引用的车辆资源，没有复制整个模型库。

原来的 `LIO-SAM`、`scout_slam_nav`、`pcd2pgm`、`fdilink_ahrs`、
`velodyne_gazebo_plugins` 和 `velodyne_simulator` 元包未移植，使用 `COLCON_IGNORE` 排除。
移除了 `robot_gazebo/CMakeLists.txt` 这个旧 catkin 工作空间生成文件；ROS 2 从各包的清单发现包。
原包中其他 ROS 1 launch、HDL-32E/T265/R200 描述及旧 C++ 插件仅作为历史源码保留，
本次启动不会使用它们。当前支持的传感器组合是 VLP-16、D435 外观的简化 RGB-D 和独立 IMU。

## 2. 第一步：理解并修改机器人模型

入口是 `scout_gazebo/urdf/base.xacro`。

- 保留原始车体网格、四个轮子和尺寸参数。
- `base_footprint` 表示机器人在地面的投影，作为根坐标系。
- `base_link` 位于底盘中心，高度由轮子半径和轮轴安装位置计算。
- 给底盘设置质量、惯量，修正盒体惯量公式。
- 轮子保持原外观安装方向，并统一实际关节轴方向。
- `move.xacro` 只使用一个 ROS 2 差速插件驱动两对轮子，避免原配置的两套运动插件冲突。

差速关系是 `v_left = v - omega * track / 2`、`v_right = v + omega * track / 2`，
轮子的角速度为对应线速度除以轮子半径。
`/cmd_vel` 中 `linear.x` 控制前后运动，`angular.z` 控制转向。
这里直接使用 Gazebo 差速插件，当前阶段不需要额外搭建 ros2_control 控制器。

`/odom` 使用轮轴编码器积分，允许存在轮胎打滑误差。
`/ground_truth/odom` 单独给出 Gazebo 的准确位姿，仅用于验证和计算误差，后续 SLAM 不应订阅它。

## 3. 第二步：接入 ROS 2 传感器

### 激光雷达

在 `velodyne_description/urdf/VLP-16.urdf.xacro` 中保留 VLP-16 的外观和 16 条竖直扫描线，
将 ROS 1 专用 Velodyne 插件替换为 `libgazebo_ros_ray_sensor.so`。
默认每圈 720 个水平采样、10 Hz、360 度、16 线、量程 0.2～20 m、竖直视场 ±15 度。
默认使用 CPU ray 传感器，避免依赖特定的 GPU 雷达渲染行为。

输出 `/velodyne_points` 是标准 `sensor_msgs/msg/PointCloud2`。
它用于算法输入，不模拟真实 Velodyne 数据包、ring 字段或逐点采样时间。
插件会丢弃部分无效返回，因此输出点数不一定等于 `16 * 720`，也不保证点云按 16 行组织。

`scout_gazebo/scripts/pointcloud_to_scan.py` 将点云中雷达坐标下
`-0.40 <= z <= 0.40 m` 的水平切片投影到二维平面，按角度保留最近距离，输出 `/scan`。
这是数据预处理，不包含建图或定位算法。
参数集中放在 `scout_gazebo/config/sensors.yaml`，扫描消息保留原点云时间戳和 `velodyne` 坐标系。
16 线雷达的最近水平射线为 ±1°，没有正好 0° 的射线；窄高度切片会排除部分远处观测。
当前切片可保留 20 m 内的 ±1° 射线，同时排除雷达下方的地面。

### 相机和 IMU

`realsense_ros_gazebo/xacro/depthcam.xacro` 保留 D435 外观，采用
`libgazebo_ros_camera.so` 的理想 RGB-D 传感器输出 640×480 彩色图、深度图和内参，目标频率 15 Hz。
RGB 和深度使用同一个对齐光学坐标系；这是供视觉实验使用的简化模型，未仿真 D435 的双目 IR、
硬件驱动、内部 IMU 或真实深度误差模型。

相机光学坐标约定为：x 向右、y 向下、z 向前。机器人坐标则是 x 向前、y 向左、z 向上。
`imu.xacro` 使用独立的 `libgazebo_ros_imu_sensor.so`，发布 100 Hz 的 `/imu/data`，
包含重力引起的加速度读数与少量测量噪声。

## 4. 第三步：准备场景和资源路径

默认启动完整版 `scout_gazebo/worlds/mapping.world`，来自课程的 `gazebo_world.zip`，
对应指导书图 9-13。`localization.world` 对应图 9-14，建筑布局相同，停放车辆有所变化。

| scene 参数 | 文件 | 场景模型数（不含机器人） | 车辆数 | AprilTag 数 |
|---|---|---:|---:|---:|
| `mapping`（默认） | `mapping.world` | 189 | 62 | 14 |
| `localization` | `localization.world` | 203 | 76 | 14 |
| `mapping_s` | `mapping_s.world` | 41 | 9 | 8 |
| `localization_s` | `localization_s.world` | 42 | 10 | 8 |

`*_s.world` 是简化小场景，不能用来对照指导书中的完整停车场。
完整版的模型数量、最终模型/链接位姿、碰撞几何和 `<state>` 保存状态保持与课程压缩包一致。
为避免 Gazebo 客户端按旧声明绘制部分静态模型，另将每个模型的初始 `<pose>` 同步为
`<state>` 中对应模型的最终位姿；不改变保存的最终布局。验收同时检查声明与保存状态是否一致。
GUI 初始相机改为俯视全景，便于与图 9-13/9-14 对照。简化版恢复为课程原文件。

当前 Gazebo 的高空俯视会让毫米级薄盒的上下表面以及重复地面发生深度冲突，出现斜条纹和标线缺失。
为解决显示问题，完整版的灰地板和薄标线 visual 改为同样平面尺寸的单面矩形，灰地板显示面位于
z=-0.02 m；默认 ground_plane 仅保留碰撞，显示使用灰地板。以上都是显示层适配，
碰撞、模型/链接位置、车辆和标签布局没有改变。

**不能直接删除 `<state>`。** 这些 world 文件的模型声明位置与编辑器最终保存位置可能不同，
`<state>` 保存了实际模型/链接位姿。删除它会造成车位、围墙、地面和标签错位。
之前的默认配置误用了简化版并删去该状态，现已纠正。
机器人、传感器检查通过，并不代表场景与教材一致；验收工具现在另外检查场景清单和关键模型实际位置。

标签从 `gazebo_apriltag_models.zip` 提取；车辆资源从 `gazebo_models.zip` 按引用提取。
启动文件设置包内 `models/` 的查找路径，机器人网格路径通过 Xacro 的 `$(find 包名)` 解析。
无需手工修改 `~/.gazebo/models`，也无需在线下载模型。

## 5. 第四步：编译并加载工作空间

在终端执行：

```bash
cd /home/ubuntu22/study/slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to scout_gazebo apriltag_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

`--packages-up-to` 会按依赖顺序构建仿真包和 AprilTag 检测包。
`--symlink-install` 让大部分资源通过符号链接安装，避免复制大型网格。
修改源文件后仍建议重新执行构建，尤其是修改 launch、CMake 或新增文件时。

每开一个新终端，都需要加载 ROS 2 和本工作空间：

```bash
cd /home/ubuntu22/study/slam
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

当前机器已经具备本阶段依赖，不需要额外安装 ROS 1 或 Cartographer。

## 6. 第五步：启动 Gazebo 和 RViz2

先在之前的 Gazebo 启动终端按 Ctrl+C，确保旧实例已经退出，然后运行：

```bash
ros2 launch scout_gazebo scout_gazebo.launch.py
```

统一入口依次启动 Gazebo、robot_state_publisher、机器人生成工具、点云转扫描节点和 RViz2。
模型生成工具等待 `/spawn_entity` 服务和 `/robot_description`，不依赖固定睡眠时间。
看到 `Successfully spawned entity [scout]` 表示生成成功。

预期画面：Gazebo 中是停车场和小车；RViz2 中是车体、彩色多线点云、红色二维扫描和相机画面。
RViz2 的 Fixed Frame 是 `odom`。当前没有建图节点，因此暂时没有 `map` 坐标系。
可在 Gazebo 左侧 World → Models 中找到 `scout`，双击定位观察。

常用参数：

```bash
# 关闭可视化窗口，适合减少界面开销；RGB-D 渲染仍需要可用的图形环境。
ros2 launch scout_gazebo scout_gazebo.launch.py gui:=false rviz:=false

# 切换到指导书图 9-14 的完整定位场景。
ros2 launch scout_gazebo scout_gazebo.launch.py scene:=localization

# 换成原始实验室场景；不要与另一个启动命令同时运行。
ros2 launch scout_gazebo scout_gazebo.launch.py \
  world:=/home/ubuntu22/study/slam/src/robot_gazebo/scout_gazebo/worlds/lab.world

# 调整机器人起点。
ros2 launch scout_gazebo scout_gazebo.launch.py x:=0.0 y:=0.0 yaw:=0.0
```

Gazebo 与 RViz2 两个窗口由同一启动命令管理。结束时，在启动终端按 Ctrl+C。
Gazebo Classic 的 EOL 提示和模型材质警告不表示机器人生成失败；真正的加载错误需要检查日志。

## 7. 第六步：用键盘控制机器人

打开第二个终端，加载上述两个环境，然后运行：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.2 -p turn:=0.4
```

焦点必须在这个终端中：`i` 前进、`,` 后退、`j` 左转、`l` 右转、`k` 停止。
按键设置的是持续速度，松开按键不会自动停车；用 `k` 停车。
正常按 Ctrl+C 退出键盘节点时也会发送零速度。

单独发送停车指令：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

## 8. 第七步：逐项检查与自动验收

先看节点和话题：

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odom --once
ros2 topic echo /imu/data --once
ros2 run tf2_ros tf2_echo odom velodyne
```

查看雷达与二维扫描频率，运行几秒后按 Ctrl+C：

```bash
ros2 topic hz /velodyne_points
ros2 topic hz /scan
```

传感器配置频率使用仿真时间。Gazebo Real Time Factor 小于 1 时，
按墙钟时间测得的频率会相应降低；自动验收使用消息中的仿真时间戳计算频率。
图像数据较大，应匹配图像发布端的 Reliable QoS；雷达可用 SensorDataQoS。

只读取数据的自动检查：

```bash
ros2 run scout_gazebo check_simulation.py
```

完整检查（会低速前进和左转，请暂时退出键盘控制节点）：

```bash
ros2 run scout_gazebo check_simulation.py --motion \
  --report /home/ubuntu22/study/slam/log/simulation_check.json
```

默认检查完整 `mapping` 场景。定位场景检查时加 `--scene localization`；简化场景对应加
`--scene mapping_s` 或 `--scene localization_s`；自定义场景可用 `--scene none` 跳过课程场景比对。
场景检查核对模型集合、车辆/标签数量、保存状态，并通过 Gazebo 原生接口读取车位、围墙、车辆、
标签的实际位置，与保存的最终位姿比对，防止仅仅模型能加载、但布局已经错位。

检查还包括模型生成、时钟、各传感器真实数据及频率、相机内参和深度有效值、16 线雷达几何、
二维扫描角度、IMU 重力、四轮关节、TF，以及真实前进、左转和零速度停车。
运动检查用 `/ground_truth/odom` 观察 Gazebo 的真实位姿，避免只看编码器变化就认为车辆移动。
程序结束发送零速度，成功输出 `RESULT: PASS`，任一失败返回非零状态。
运动检查会改变小车位置；需要回到起点时结束仿真并重新启动。

## 9. AprilTag 检测（指导书第 5 节，ROS 2 版本）

优先使用课程资源：`apriltag/` 提取自 `实验9相关文件/apriltag.zip`，版本 3.4.2；
`apriltag_ros/msg/` 和 LICENSE 来自课程 `apriltag_ros.zip`。
课程 ROS 1 包依赖 catkin/nodelet，无法直接用于 Humble，因此保留其检测消息定义，
重新实现 ROS 2 连续检测适配。此处的 `apriltag_ros` 是本课程的局部移植，
只支持独立 `tag36h11`，未实现原包的标签 bundle、单张图片服务或 Cartographer landmark。
不要再安装另一个同名的 `apriltag_ros` 包覆盖它。

库由 colcon 安装到工作空间 `install/apriltag`，无需 `sudo make install`。
源码、参数和启动文件都在 `src/robot_gazebo` 内，课程压缩包保持不变。
库源码中的示例、测试和许可证保留原样；构建和运行报告统一放在已有的 build/install/log 中。

按第 5 节重新编译后，在每个终端加载 ROS 2 和工作空间环境。
先保持原有仿真和键盘控制运行，在第三个终端执行：

```bash
ros2 launch apriltag_ros continuous_detection.launch.py
```

配置在 `apriltag_ros/config/tags.yaml`：标签族固定为 `tag36h11`，
`tag_size: 0.24` 单位为米，配置 ID 为 0～15。
场景中的标签平面宽 0.30 m，其中外围白边不参与测距，黑色方框边长为 0.24 m。
`tag_ids` 是允许的 ID 列表，空列表允许该族所有 ID；所有标签使用同一边长。
`threads`、`quad_decimate`、`max_hamming` 可调，修改参数文件后重启检测节点。

节点读取 `/camera/color/image_raw` 和 `/camera/color/camera_info`，
缓存该仿真相机的固定内参并检查图像尺寸、坐标系。
Gazebo 相机无畸变，因此可直接使用 raw 图像；真实畸变图像应先校正。
输入采用 Best Effort、深度 1，兼容相机的 Reliable 发布端，避免处理积压的旧帧。
所有输出使用原始图像时间戳，启用 `use_sim_time`。

| 输出 | 内容 |
|---|---|
| `/tag_detections` | `apriltag_ros/msg/AprilTagDetectionArray`，包含 ID、尺寸和相机坐标系下的位姿 |
| `/tag_detections_image` | `sensor_msgs/msg/Image`，绿色框和红色 ID，未检测到标签时仍输出图像 |
| `/tf` | `camera_color_optical_frame → tag_<ID>`，仅在本帧检测到该标签时发布 |

位姿中 x 向右、y 向下、z 向前，单位为米。
`pose.pose.covariance` 全零表示此适配器未估计协方差，不能作为精确零误差使用。
无标签时 `/tag_detections` 发布空数组，这是正常输出；历史 TF 不代表当前仍能看见标签。

第四个终端查看检测消息：

```bash
ros2 topic echo /tag_detections
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag_0
```

通过键盘将相机朝向柱子上的标签，靠近到约 1～3 m 便于观察。
已有 RViz2 配置新增了 **AprilTag Detection** 图像显示，话题为 `/tag_detections_image`。
若 RViz2 是修改配置前打开的，重启仿真或手动 Add → Image 并选择该话题。
开启 TF 显示可观察 `tag_<ID>` 坐标系；它表示视觉测得的位置，而非地图中的已知地标位置。

只读取数据的验收（需要当前画面能看到标签）：

```bash
ros2 run apriltag_ros check_apriltag.py --timeout 30 \
  --report-dir /home/ubuntu22/study/slam/log/apriltag_validation/manual
```

可加 `--expected-id 0` 检查指定标签。程序验证位姿数值、单位四元数、尺寸、
同时间戳检测与图像、TF 与位姿一致，以及图像上存在标注；成功输出 `RESULT: PASS`。
报告目录生成 `check.json` 和 `detection.png`。

已在完整版 `mapping.world` 实测：

- ID 0：最终构建复验约 1.789 m，检测、图像和 TF 验收通过。
- ID 7：相机前方约 1.860 m，检测、图像和 TF 验收通过。
- 将相机转向无标签区域：连续 5 帧为空检测，图像继续输出。

对应证据位于 `log/apriltag_validation/tag0/`、`tag7/`、`no_tag.json`。
这是检测链路验证，未对整个停车场的识别率或运动中定位精度做评估。
复现 ID 0 的观察位置时，先结束旧仿真，再启动：

```bash
ros2 launch scout_gazebo scout_gazebo.launch.py x:=-44.1681 y:=1.5 yaw:=1.57079632679
```

然后启动检测节点，即可直接看到柱子上的 ID 0。正常实验仍可使用原来的默认起点。

## 10. Cartographer 建图定位（指导书第 6 节，ROS 2 版本）

### 依赖和课程文件

配置以 `实验9相关文件/mapping.lua`、`localization.lua`、两个 launch 和
`continuous_detector.cpp` 为基础移植；课程压缩包和原文件不修改。
Cartographer 使用 ROS 2 Humble 官方二进制，参数、启动文件和辅助脚本集中在
`scout_gazebo/config`、`launch`、`scripts`，不再建立嵌套工作空间。

本机没有可用的 sudo 密码，依赖已解包在 `install/cartographer_runtime`，
由 scout_gazebo 的环境钩子加载。每次 `source install/local_setup.bash` 即可使用，
无需单独设置一串环境变量。版本记录在 `install/cartographer_runtime/packages.json`。
若删除 install，需要在工作空间根目录重新执行：

```bash
python3 src/robot_gazebo/scout_gazebo/scripts/setup_cartographer.py
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to scout_gazebo --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

新机器也可以使用系统安装的 `ros-humble-cartographer`、`ros-humble-cartographer-ros`；
不需要同时配置本地和系统两套依赖。

### 建图

每个终端先执行本 README 第 5 节的 source 命令。先结束旧的仿真、检测和算法节点，
避免同一话题、TF 或运动命令有重复发布者。

终端 1：启动指导书完整版建图场景，关闭基础 RViz，交给建图入口打开地图视图。

```bash
ros2 launch scout_gazebo scout_gazebo.launch.py scene:=mapping rviz:=false
```

终端 2：

```bash
ros2 launch scout_gazebo mapping.launch.py
```

这个入口会自动启动 AprilTag、landmark 转换、Cartographer、5 cm 栅格地图和 RViz2。
不要再单独启动第二个 AprilTag 节点。若它已经在运行，可加 `apriltag:=false`。
不需要重复启动 pointcloud_to_laserscan：基础仿真已经把 VLP-16 转成 `/scan`。
无界面检查可加 `rviz:=false`。

终端 3：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.4
```

沿停车场通道移动，转弯放慢速度，经过不同柱子观察 AprilTag，最后返回已走过的区域形成回环。
只在起点旋转无法覆盖整个停车场；没有看到的墙后区域在地图中保持未知是正常的。
RViz2 的 Fixed Frame 为 `map`，可观察 Map、Scan、Trajectory、AprilTag landmarks。
`/landmark_poses_list` 的 MarkerArray 显示的是优化后的地标。

关键接口：

```bash
ros2 topic echo /landmark
ros2 topic echo /tracked_pose
ros2 run tf2_ros tf2_echo map base_footprint
ros2 service call /get_trajectory_states cartographer_ros_msgs/srv/GetTrajectoryStates '{}'
```

TF 关系为 `map → odom → base_footprint → sensors`。
Gazebo 已发布 odom 到底盘，因此使用 `provide_odom_frame=false`，避免指导书旧配置产生重复 TF。
`tracking_frame=imu_base` 与 IMU 原点重合，`use_odometry=false` 与课程配置一致。
`publish_tracked_pose=true` 直接在 Lua 中启用，无需修改系统头文件。

`apriltag_landmarks.py` 按图像时间戳把相机坐标下的 tag 位姿转换到 `imu_base`，
发布正式的 `cartographer_ros_msgs/msg/LandmarkList` 到 `/landmark`，ID 和权重沿用课程示例。
无需复制或伪造 Cartographer 的消息头文件。无 TF 时跳过观测；无标签时发布空数组。

### 保存地图

先在键盘终端按 `k` 停车，保持仿真与建图节点运行，在第四个终端执行：

```bash
ros2 run scout_gazebo save_map.py --map /home/ubuntu22/study/slam/maps/parking
```

工具按指导书顺序结束活动轨迹、调用 `/write_state`，再用官方
`cartographer_pbstream_to_ros_map` 导出 0.05 m 栅格，生成同名前缀的三个文件：

- `parking.pbstream`：轨迹、子图、约束和地标，用于 Cartographer 定位。
- `parking.pgm`：栅格图像。
- `parking.yaml`：分辨率、原点和图像路径，用于 map_server。

保存操作会结束本次建图轨迹，之后请退出建图入口；继续开车不会再更新这条已结束轨迹。
默认拒绝覆盖已有文件，需要覆盖时显式加 `--overwrite`。
地图保存于工作空间 `maps/`，不放在源码或日志目录。

### 纯定位

结束建图和旧仿真，然后启动指导书车辆分布改变后的定位场景：

```bash
ros2 launch scout_gazebo scout_gazebo.launch.py scene:=localization rviz:=false
```

另一个终端加载刚保存的地图（参数是**不带扩展名的绝对路径**）：

```bash
ros2 launch scout_gazebo localization.launch.py map:=/home/ubuntu22/study/slam/maps/parking
```

入口将 pbstream 作为冻结轨迹加载；采用 `pure_localization_trimmer` 保留最多 3 个活动子图。
定位用的栅格由 Nav2 map_server 读取已保存的 yaml，生命周期管理器自动激活。
按指导书的纯定位要求，此模式不启动 Cartographer occupancy_grid 节点，`/map` 始终是保存的地图。

默认起点与建图起点相同便于首次验证。换起点可给仿真传入 `x/y/yaw`；
刚启动时 `map → odom` 可能尚未校正，不能把最初几秒的位置当成已收敛的定位结果。
缓慢运动并观察扫描是否贴合地图、Trajectory 是否位于正确通道。
停车场有重复柱子和停车位，若全局匹配尚未收敛或选错通道，
使用 RViz2 工具栏 **2D Pose Estimate** 在地图上给出大致位置和朝向。
`initial_pose.py` 会通过官方服务结束活动定位轨迹并启动带初始位姿的新轨迹，冻结地图保持加载。
操作时在地图上按下小车的大致位置，按住鼠标拖动箭头指向车头，再松开。
这份 `maps/parking` 的首次定位建议使用默认仿真起点；换起点时建议主动设置初始位姿，
不要完全依赖重复停车位环境中的无初值全局匹配。

现有地图的实测建图记录：约 316 m 闭环行驶、12 个标签、全程位置 RMSE 0.121 m，
末段平均位置误差 0.025 m。不同起点的定位测试已验证地图加载、landmark 输入和轨迹重启，
初次无初值测试的末段位置误差约 0.8 m，未达到验收脚本的 0.5 m 阈值。
不能把建图精度当作任意起点的定位保证；按当前操作流程使用近似初始位姿后再观察扫描重合情况。
另有一项已知限制：当前官方二进制在纯定位启用地标和轨迹裁剪后，
测试中按 Ctrl+C 退出曾触发 `id.h` 断言。此处尚未修复上游退出路径，
若终端显示该错误，确认旧进程退出后再重新启动；不要把它当作正常退出验收通过。

### 可复现验收

只读定位误差检查（需要地图对应同一世界，并使用建图测试生成的对齐报告）：

```bash
ros2 run scout_gazebo check_cartographer.py \
  --reference /home/ubuntu22/study/slam/log/cartographer_validation/mapping.json \
  --duration 60 --report /home/ubuntu22/study/slam/log/cartographer_validation/localization.json
```

重新进行完整建图行驶测试时，先使用默认起点启动 mapping 场景和建图入口，退出键盘控制，执行：

```bash
ros2 run scout_gazebo check_cartographer.py --drive \
  --report /home/ubuntu22/study/slam/log/cartographer_validation/mapping.json
```

`--drive` 会沿完整停车场已选通道行驶并停车观察标签，最后返回起点，耗时随仿真实时率变化。
它使用 `/ground_truth/odom` 控制测试路径和评估误差；Cartographer 不订阅该话题。
建图对齐只使用开始时的一对位姿，后续误差不会逐帧重新对齐；
定位复用同一固定对齐关系，避免以重新拟合来掩盖定位错误。
报告包含轨迹、各检查点、位置 RMSE、末段误差、地图有效栅格和观测标签 ID。

## 11. 后续自己写算法时使用的接口

| 话题 | 类型 | 用途 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 机器人运动控制 |
| `/odom` | `nav_msgs/msg/Odometry` | 编码器积分的运动预测 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 四轮实际转角和动态 TF |
| `/velodyne_points` | `sensor_msgs/msg/PointCloud2` | 16 线雷达点云 |
| `/scan` | `sensor_msgs/msg/LaserScan` | 二维建图/定位输入 |
| `/imu/data` | `sensor_msgs/msg/Imu` | 姿态、角速度、加速度 |
| `/camera/color/image_raw` | `sensor_msgs/msg/Image` | AprilTag 视觉输入 |
| `/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | 米制深度图，32FC1 |
| `/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | 深度相机内参 |
| `/ground_truth/odom` | `nav_msgs/msg/Odometry` | 仅供算法误差评估 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时间 |

当前 TF 是 `odom → base_footprint → base_link → 轮子/雷达/相机/IMU`。
后续建图/定位节点应设置 `use_sim_time:=true`，并负责发布 `map → odom`。
保持同一个 TF 子坐标系只有一个发布者；不要让自己的算法重复发布底盘或传感器的静态 TF。

## 本机验收记录（机器人初版，修正场景前）

已实际启动停车场、Scout、Gazebo 和 RViz2，完成 **32 项检查，全部通过**。
该轮检查针对旧的简化场景，不能作为教材场景一致性的证据。
修正后的完整场景检查报告位于工作空间 `log/simulation_check.json`，以报告中的结果为准。

- 检测到 16 个雷达竖直扫描角，二维扫描输出 720 个角度格。
- 实测雷达约 9.65 Hz、二维扫描约 9.99 Hz、
  IMU 约 99.68 Hz、轮式里程计 50.00 Hz。
- 启动后的采样窗口中 RGB/深度图像约 13.59 Hz，
  配置目标为 15 Hz；图像和内参均收到有效数据。
- 前进测试的实际位移 0.504 m。
- 左转测试的实际偏航变化 0.548 rad，约 31.4°。
- 零速度停车检查通过；底盘、轮子、雷达、相机、IMU 的 TF 链完整。

以上频率按仿真时间统计，不能直接等同于墙钟时间的处理吞吐量。
四轮滑移转向会使编码器积分与真实转角产生差异，后续融合算法可利用 IMU、激光和视觉校正。

## 12. 参考与资源来源

- 课程 PDF《实验9 移动机器人建图定位仿真实验》及同目录配套压缩包。
- 原始机器人仓库：https://github.com/linzs-online/robot_gazebo
- ROS 2 Gazebo 插件：https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2/gazebo_plugins
- 原资源中各网格、模型和源码的版权仍归对应作者；车轮文件保留了原始 BSD 版权声明。
