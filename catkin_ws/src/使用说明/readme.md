#文件结构介绍：
c_lightlot : 前期练习使用，无需学习，可直接删除
robot_bag : 用于存放实验数据，添加实验时，在内部添加实验即可
robot_controller ：可在内部写控制算法，如PID控制等 #当前并未使用
robot_controller： URDF文件存储位置，每个机器人应保存为一个独立的包
robot_driver ： 内部存放通讯代码，包括EtherCAT、CANopen、TCP\IP通讯代码
robot_driver： gazebo物理仿真和参数文件
robot_gui：图形界面，当前并未使用该文件，将来可以图形界面存储在该文件
robot_kinematic：当前并未使用，内部有一一些KDL库C++代码示例、moveit使用示例
robot_moveit：用于直接调用ROS自带运动学和代码，有一些moveit示例
robot_msg： 自定义数据结构，当前并未使用
robot_python: Python编写的主要代码存储在该文件
robot_sensor：传感器先关通讯代码，包括串口和TCP/IP通讯
robot_sensor：工艺代码文件，当前并未使用，每部有打磨工艺代码
robot_visual：视觉文件，当前存储机械臂抓取相关代码
图片和视频：存储实验相关的截图
#软件依赖
Python2.7一般系统自带
QT、PyQt库、pyqtgraph库，及头文件包含的相关库
ros_ethercat-master：文件中介绍了安装EtherCAT包相关依赖
CANopen_v1-master：CANopen_v1-master/testcan/readme文件中存储CANopen依赖文件及安装方法
#软件启动
机械臂下位机：
d@f:~$ sudo su #密码6
root@f:/home/d# source .bashrc
root@f:/home/d# roslaunch rqt_ethercat_test_plugin ethercat_test.launch 
注：ethercat_test.launch为EtherCAT调试启动文件
注：econtroller_manager_test.launch 为s实际运行启动文件
root@f:/home/d# roslaunch testcan CANopenMaste_command_jointStates.launch
注：CANopenMaste_command_jointStates.launch为与界面连接并转发的关节角的启动文件
注：CANopenMaster.launch为原始启动文件，每个关节角一个独立控制器
ur5_ros_master.py文件在UR5文件中，可以启动UR5机械臂与上位机通信
六维力传感器：
连接后线，上电后，在robot_sensor文件中的force_sensor功能包中
启动tcpsensor_armc.py等对应传感器即可
软件界面使用：
(1)启动机械臂下位机
(2)启动六维力传感器（非必要）
(3)运行：gui_main.py文件














