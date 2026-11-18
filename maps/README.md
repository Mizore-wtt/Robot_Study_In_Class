# 停车场地图

`parking.pbstream`、`parking.pgm`、`parking.yaml` 由本工程中的 Cartographer 实际建图生成，
对应课程完整版 `mapping.world`，栅格分辨率 0.05 m。

小车通过 `/cmd_vel` 沿通道真实行驶约 316 m 并返回起点，观察到 12 个 AprilTag。
建图未使用 Gazebo 真值作为算法输入。真值仅用于测试路径控制和独立误差评估。

完整轨迹验证报告：`../log/cartographer_validation/mapping.json`。

直接加载定位：

```bash
ros2 launch scout_gazebo localization.launch.py map:=/home/ubuntu22/study/slam/maps/parking
```

先启动基础仿真，使用相同建筑布局的 `scene:=localization` 可以验证车辆变化后的定位。
地图边缘与车辆后的灰色区域是未充分观测区域；移动继续建图可增加覆盖。

不要编辑 pbstream 或手动画入车辆。重新建图应通过 `save_map.py` 导出三个相匹配的文件。
当前 yaml 的图像路径为本机绝对路径；移动地图目录后需要同步修改 image 字段。
