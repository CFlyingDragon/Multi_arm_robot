#创建人陈永厅
#文件结构如下
robot_python/
    CMakesLists.txt
    package.xml
    launch/ # roslaunch files go here
    msg/ # msg files go here
    nodes/ # installable python nodes go here
    src/ # this is where your python modules exported as libraries or used in your nodes go.
    scripts/ # generally non-exported python scripts -- stuff that might be called to setup your package and code gen stuff
    srv/ # service descriptions go here
    __init__.py # only necessary if you want to use anything from the scripts directory in your package src and node files
    setup.py # more on this later

#src:编写的源代码(基本功能模块）放在src文件的子文件中
(1)运动学模块
(2)规划模块
(3)导纳控制模块
#nodes:可执行文件放在nodes文件中，其调用src中功能函数
存储一些脚本启动的ros节点文件

#自定义的数据类型放在msg、srv中

#运行所产生的数据放在文件data中

#scripts存放简单独立的功能或实验模块
#demo文件存储一些示例代码，主要是用于学习相关的知识
#gui文件
(1)存储UI生成的***.py文件
(2)存储界面调用函数gui_main.py 及***_sub*.py子界面
#ui文件
采用绘图功能工具绘制的文件
自动生成的****.py文件，需要复制到gui文件中
#test文件
一些函数是否正确的调试代码

