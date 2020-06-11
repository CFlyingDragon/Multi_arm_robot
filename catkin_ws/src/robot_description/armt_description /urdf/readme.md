#用于物理仿真的模型文件
1.**.gzaebo文件
用于配置gazebo的插件和属性，例如配置力传感器等
2.*_transmission.xacro文件
 transmission是硬件抽象层中的一部分, transmission的任务很清晰, 主要是两个方向的任务:
- Actuator <–> Joint: 定义Joint的Pos, Vel, Tor与Motor的Pos, Vel, Tor.
- State: 将编码器(例如角度, 力矩传感器等)的数据, 传递给对应的Joint, 变成最终的RobotState.
注：本文直接主要配置两个，关节控制类型hardware_interface和传动比，其中传动比直接设置为1，Actuator执行器(动力源）
3.**.xacro机器人模型文件
机器人模型文件，由solidworks导出的**.urdf直接编而得，用于描述机器人连杆和关节位置属性
用于调动前2个文件


