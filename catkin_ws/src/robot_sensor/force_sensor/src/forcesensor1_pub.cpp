#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <stdio.h>
#include <stdlib.h>
#define pi 3.1415926
using namespace std;
using namespace boost::asio;

float ByteToFloat(char* byteArry){	//使用取地址的方法进行处理
	return *((float*)byteArry);
}
int main(int argc, char* argv[]){
	std::vector<double> testArray = {1,2,3,4,5,6};
	std_msgs::Float64MultiArray msg;

    //设置串口相关东西
	io_service iosev;
	boost::asio::serial_port *SerialPort;
    serial_port sp(iosev, "/dev/forcesensor1");	//节点文件 端口名，通过文件配置保证其确定性
	sp.set_option(serial_port::baud_rate(115200));	// 设置参数
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none)); //控制流
    sp.set_option(serial_port::parity(serial_port::parity::none)); // 奇偶校验，可以为serial_port::parity::none / odd / even
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));//停止位
    sp.set_option(serial_port::character_size(8));//字符大小

    //节点相关信息
	ros::init(argc, argv, "forcesensor1_pub");
  	ros::NodeHandle n;
	ros::Publisher force_pub = n.advertise<std_msgs::Float64MultiArray>("six_axis_force_1", 1000);
	ros::Rate loop_rate(100);

    //力传感器指令
	char open[8] = {0x41,0x54,0x2B,0x47,0x4F,0x44,0x0D,0x0A};
	boost::system::error_code err;
    boost::asio::streambuf ack_buf;                                                             //返回缓存
    std::string set_update_rate = "AT+SMPR=100\r\n";                                            //设置采样频率
	// std::string set_decouple_matrix = "(-1.66886,0.45468,2.40226,116.59568,8.04463,-114.59075);(-1.19954,-134.46625,-4.67941,67.71708,6.35921,64.98772);(330.02844,-0.00649,334.45620,2.43349,329.83661,-1.25213);(-0.18771,-0.08793,-11.54467,0.00086,11.44153,0.19783);(13.72324,-0.16868,-6.47828,-0.08208,-6.25346,0.01777);(0.09735,4.92084,0.05934,4.70998,0.05902,4.36001)\r\n";
    std::string set_compute_unit = "AT+DCPCU=MVPV\r\n";                                         //设置矩阵运算单位
    std::string set_recieve_format= "AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n";        //上传数据格式
    std::string get_data_stream = "AT+GSD\r\n";                                                 //连续上传数据包
    std::string stop_data_stream = "AT+GSD=STOP\r\n";                                           //
    write(sp, boost::asio::buffer(set_update_rate));                           //写入更新率
	std::cout<<set_update_rate<<std::endl;
    read_until(sp, ack_buf, "\r\n");                                           //读取函数，读到分隔符为止
	std::istream is(&ack_buf);
	// write(sp, boost::asio::buffer(set_decouple_matrix));
	// read_until(sp, ack_buf, "\r\n");
	// std::cout<<set_decouple_matrix<<std::endl;
    write(sp, boost::asio::buffer(set_compute_unit));                          //设置计算单位
	read_until(sp, ack_buf, "\r\n");
    write(sp, boost::asio::buffer(set_recieve_format));                        //设置接受格式
	read_until(sp, ack_buf, "\r\n");
	std::cout<<set_recieve_format<<std::endl;
    write(sp, boost::asio::buffer(get_data_stream));                           //获得连续数据
	std::cout<<get_data_stream<<std::endl;
	int count = 0;
	int count_initforce = 0;
	double initforce[6] = {0};
	while (ros::ok()){
		char data_frame[31];
		std::string data_str;
        size_t len = read(sp, buffer(data_frame,31));                         //接受返回数据
		char data_1[4] = {data_frame[6],data_frame[7],data_frame[8],data_frame[9]};
		char data_2[4] = {data_frame[10],data_frame[11],data_frame[12],data_frame[13]};
		char data_3[4] = {data_frame[14],data_frame[15],data_frame[16],data_frame[17]};
		char data_4[4] = {data_frame[18],data_frame[19],data_frame[20],data_frame[21]};
		char data_5[4] = {data_frame[22],data_frame[23],data_frame[24],data_frame[25]};
		char data_6[4] = {data_frame[26],data_frame[27],data_frame[28],data_frame[29]};
        double f_x = ByteToFloat(data_1);                                    //将ASC码转为浮点数
		double f_y = ByteToFloat(data_2);
		double f_z = ByteToFloat(data_3);
		double m_x = ByteToFloat(data_4);
		double m_y = ByteToFloat(data_5);
		double m_z = ByteToFloat(data_6);
        if(count_initforce < 10){                                           //累加10次，计算初始平均值
			initforce[0] += f_x;initforce[1] += f_y;initforce[2] += f_z;
			initforce[3] += m_x;initforce[4] += m_y;initforce[5] += m_z;
			count_initforce++;
		}
        if(count_initforce == 10){                                          //10次后，受力减初始值
			testArray[0] = f_x - initforce[0]/10; testArray[1] = f_y - initforce[1]/10; testArray[2] = f_z - initforce[2]/10;
			testArray[3] = m_x - initforce[3]/10; testArray[4] = m_y - initforce[4]/10; testArray[5] = m_z - initforce[5]/10;
			msg.data = testArray;
			force_pub.publish(msg);
		}
		count++;
		if(count == 50){
			ROS_INFO("%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f;\n ",testArray[0],testArray[1],testArray[2],testArray[3],testArray[4],testArray[5]);
			count = 0;
		}
		iosev.run();
		loop_rate.sleep();
	}
	write(sp, boost::asio::buffer(stop_data_stream));
	iosev.stop();
	return 0;
}



/*#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
# include <stdio.h>
# include <stdlib.h>
#define pi 3.1415926

using namespace std;
using namespace boost::asio;

int force_count_10 = 0;
unsigned char buf[19]={0};
unsigned char torque_buf[12]={0};//采集的原始数据
float six_axis_force[6] = {0.0,0.0,0.0,0.0,0.0,0.0};//计算的结果
float six_axis_force_init[6] = {0};
float six_axis_force_modify[6] = {0};
float AmpZero[6] = {32817.0,32768.0,32808.0,32742.0,32756.0,32724.0};
float Gain[6] = {123.905590,123.994933,123.994933,123.988551,123.889636,124.052367};
float Ex[6] = {4.985400,4.985400,4.985400,4.985400,4.985400,4.985400};

float decoupled_matrix[6][6]={
-1.66886,  0.45468,     2.40226,   116.59568, 8.04463,   -114.59075,
-1.19954,  -134.46625,  -4.67941,  67.71708,  6.35921,   64.98772,
330.02844, -0.00649,    334.45620, 2.43349,   329.83661, -1.25213,
-0.18771,  -0.08793,    -11.54467, 0.00086,   11.44153,  0.19783,
13.72324,  -0.16868,    -6.47828,  -0.08208,  -6.25346,  0.01777,
0.09735,   4.92084,     0.05934,   4.70998,   0.05902,   4.36001
};

int check_sum(unsigned char *buf){
	int sum = 0;
	for(int i = 6;i < 18;i++){
		sum += buf[i];
		//printf("sum = %d,buf[i] = %d\n",sum,buf[i]);
	}
	//printf("int sum:%d\n",sum);
	sum = (unsigned char) sum;
	//printf("buf[18] = %d,char sum=%d\n",buf[18],sum);
	if(sum == buf[18])
		return 1;
	else
		return 0;
}

int six_axis_calculation(void){
	int returnflag = 0;
	if(check_sum(buf)){
		float six_axis_data[6] = {0.0,0.0,0.0,0.0,0.0,0.0};
		for(int i = 0;i < 6;i++)
			six_axis_force[i] = 0.0;
		for(int i = 0;i < 6;i++){
			six_axis_data[i] = 1000.0*((int)(torque_buf[i*2]<<8|torque_buf[i*2+1]) - AmpZero[i])/65535.0*5/Gain[i]/Ex[i];
		}
		for(int i = 0;i < 6;i++){
			for(int j = 0;j < 6;j++)
				six_axis_force[i] += decoupled_matrix[i][j]*six_axis_data[i]; 
		}
		force_count_10++;
		if(force_count_10 < 10){
			for(int i = 0;i < 6;i++)
				six_axis_force_init[i] = - six_axis_force[i];
		}
		else{
			force_count_10 = 10;
			for(int i = 0;i < 6;i++)
				six_axis_force_modify[i] = six_axis_force[i] + six_axis_force_init[i];
			returnflag = 1;
		}
		}
	else
		returnflag = 0;
	
	return returnflag;
}

int main(int argc, char* argv[])
{
	std::vector<double> testArray = {1,2,3,4,5,6};
	std_msgs::Float64MultiArray msg;
	int time = 0;

	io_service iosev;

	boost::asio::streambuf ack_buf;
	//节点文件
	serial_port sp(iosev, "/dev/ttyUSB0");
	// 设置参数
	sp.set_option(serial_port::baud_rate(115200));
	sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
	sp.set_option(serial_port::parity(serial_port::parity::none));
	sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	sp.set_option(serial_port::character_size(8));

	ros::init(argc, argv, "forcesensor1_pub");
  	ros::NodeHandle n;
	ros::Publisher force_pub = n.advertise<std_msgs::Float64MultiArray>("six_axis_force_1", 1000);
	ros::Rate loop_rate(100);

	//char open[8]={0x41,0x54,0x2B,0x47,0x4F,0x44,0x0D,0x0A};
	string receive_format="AT+SGDM=(A01,A02,A03,A04,A05,A06);C;1;(WMA:1,1,1,2,4)\r\n";
	write(sp,buffer(receive_format));
	read_until(sp,ack_buf,"\r\n");

	string update_rate="AT+SMPR=100\r\n";
	write(sp,buffer(update_rate));
	read_until(sp,ack_buf,"\r\n");

	char open1[8]={0x41,0x54,0x2B,0x47,0x53,0x44,0x0D,0x0A};
	write(sp, buffer(open1, 8));
	while (ros::ok()){

		read(sp, buffer(buf,19));

		for(int i = 0;i < 12;i++)
			torque_buf[i] = buf[i+6];

		if(six_axis_calculation()){

			ROS_INFO("Force:%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f ",
					 six_axis_force_modify[0],six_axis_force_modify[1],six_axis_force_modify[2],
					 six_axis_force_modify[3],six_axis_force_modify[4],six_axis_force_modify[5]);
			for(int i = 0;i < 6;i++)
				testArray[i] = six_axis_force_modify[i];	
			msg.data = testArray;
			force_pub.publish(msg);
		}
		iosev.run();
		loop_rate.sleep();
		}
	string stop="AT+GSD=STOP\r\n";
	write(sp,buffer(stop));

	iosev.stop();

	return 0;
}*/
