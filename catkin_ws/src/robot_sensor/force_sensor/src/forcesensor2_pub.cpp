#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
# include <stdio.h>
# include <stdlib.h>
#define pi 3.1415926

using namespace std;
using namespace boost::asio;


float ByteToFloat(char* byteArry)//使用取地址的方法进行处理
{
return *((float*)byteArry);
}

int main(int argc, char* argv[])
{
	std::vector<double> testArray = {1,2,3,4,5,6};
	std_msgs::Float64MultiArray msg;

	io_service iosev;
	boost::asio::serial_port *SerialPort;

	//节点文件
    serial_port sp(iosev); //, "/dev/forcesensor2");
	// 设置参数
	sp.set_option(serial_port::baud_rate(115200));
	sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
	sp.set_option(serial_port::parity(serial_port::parity::none));
	sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	sp.set_option(serial_port::character_size(8));

	ros::init(argc, argv, "forcesensor2_pub");
  	ros::NodeHandle n;
	ros::Publisher force_pub_2 = n.advertise<std_msgs::Float64MultiArray>("six_axis_force_2", 1000);
	ros::Rate loop_rate(100);

	char open[8]={0x41,0x54,0x2B,0x47,0x4F,0x44,0x0D,0x0A};
	boost::system::error_code err;
	boost::asio::streambuf ack_buf;
	std::string set_update_rate = "AT+SMPR=100\r\n";
	// std::string set_decouple_matrix = "AT+DCPM=(-5.33036,0.36865,8.12359,113.63326,-1.80702,-116.33033);(-3.43261,-132.17362,1.42395,66.06570,-1.17770,66.36919);(326.72946,1.21139,335.66754,0.75826,339.12070,-1.51955);(0.22493,-0.02551,-11.52057,-0.02016,11.79001,-0.01044);(13.54671,0.00529,-6.52430,0.00249,-6.45208,-0.02931);(0.03690,4.85712,0.05495,4.69032,0.07504,4.34013)\r\n";
	std::string set_compute_unit = "AT+DCPCU=MVPV\r\n";
	std::string set_recieve_format= "AT+SGDM=(A01,A02,A03,A04,A05,A06);E;1;(WMA:1)\r\n";
	std::string get_data_stream = "AT+GSD\r\n";
	std::string stop_data_stream = "AT+GSD=STOP\r\n";
	write(sp, boost::asio::buffer(set_update_rate));
	std::cout<<set_update_rate<<std::endl;
	read_until(sp, ack_buf, "\r\n");
	std::istream is(&ack_buf);

	write(sp, boost::asio::buffer(set_compute_unit));
	read_until(sp, ack_buf, "\r\n");

	write(sp, boost::asio::buffer(set_recieve_format));
	read_until(sp, ack_buf, "\r\n");
	std::cout<<set_recieve_format<<std::endl;

	write(sp, boost::asio::buffer(get_data_stream));
	std::cout<<get_data_stream<<std::endl;

	int count = 0;
	int count_initforce = 0;
	double initforce[6] = {0};
	while (ros::ok()){
		char data_frame[31];
		std::string data_str;
		size_t len = read(sp, buffer(data_frame,31));

		char data_1[4] = {data_frame[6],data_frame[7],data_frame[8],data_frame[9]};
		char data_2[4] = {data_frame[10],data_frame[11],data_frame[12],data_frame[13]};
		char data_3[4] = {data_frame[14],data_frame[15],data_frame[16],data_frame[17]};
		char data_4[4] = {data_frame[18],data_frame[19],data_frame[20],data_frame[21]};
		char data_5[4] = {data_frame[22],data_frame[23],data_frame[24],data_frame[25]};
		char data_6[4] = {data_frame[26],data_frame[27],data_frame[28],data_frame[29]};
		double f_x = ByteToFloat(data_1);
		double f_y = ByteToFloat(data_2);
		double f_z = ByteToFloat(data_3);
		double m_x = ByteToFloat(data_4);
		double m_y = ByteToFloat(data_5);
		double m_z = ByteToFloat(data_6);
		if(count_initforce < 10)
		{
			initforce[0] += f_x;initforce[1] += f_y;initforce[2] += f_z;
			initforce[3] += m_x;initforce[4] += m_y;initforce[5] += m_z;
			count_initforce++;
		}
		if(count_initforce == 10)
		{
			testArray[0] = f_x - initforce[0]/10; testArray[1] = f_y - initforce[1]/10; testArray[2] = f_z - initforce[2]/10;
			testArray[3] = m_x - initforce[3]/10; testArray[4] = m_y - initforce[4]/10; testArray[5] = m_z - initforce[5]/10;
			msg.data = testArray;
			force_pub_2.publish(msg);
		}
		count++;
		if(count == 50)
		{
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
