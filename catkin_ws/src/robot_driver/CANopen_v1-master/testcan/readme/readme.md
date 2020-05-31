#配置底层
将cyt中的三个文件拷贝到系统文件\lib中
#调零位步骤
1.rqt调到合适位置
2.读取canopenexample节点中的反馈位置
3.改canopen_vci_ros.h文件中CANopenVCIROS::init_structure_motor_pos
的前7个
