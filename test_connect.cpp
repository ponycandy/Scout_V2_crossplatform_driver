
#include <test_connect.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <boost/thread.hpp>


test_connect::test_connect(int baud) :Connector(baud), io_context_()
{

}

int test_connect::Send(const void* buf, const int buflen)
{
	boost::asio::const_buffer buffer(buf, buflen);
    // return boost::asio::write(Core_socket, buffer);
    return Core_socket->write_some(buffer);

}

int test_connect::init(std::string Ipaddress, int port)
{
	Core_socket = new boost::asio::ip::tcp::socket(io_context_);
	boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(Ipaddress), port);

	Core_socket->connect(endpoint);
	internel_socket = Core_socket;

	unsigned char strbuffer[13] = { 0x08,0x00,0x00,0x04,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };
	Send(strbuffer, 13);
	unsigned char strbuffer0[13] = { 0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00 };

	unsigned char strbuffer1[13] = { 0x08,0x00,0x00,0x01,0x21,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00 };

	unsigned char strbuffer2[13] = { 0x08,0x00,0x00,0x01,0x21,0x01,0x01,0x05,0x01,0x05,0x00,0x00,0x00 };

	//char strbuf1[8] {\000,\000,\000,\000,\000,\000,\000,\b};
	Send(strbuffer0, 13);
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	Send(strbuffer1, 13);
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	Send(strbuffer2, 13);
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	printf("initialization completed. \n");
    return 1;
}


void test_connect::unpack_all() //打印一次所有信息，调试专用函数
{
}





void test_connect::printall()//打印一次所有信息，调试专用函数
{

}






void test_connect::cmd_test()
{
	//灯光控制指令,所有指令按照以下标准赋值，不要按照ugvsdk里面到代码赋值
	int i{ 0 };
	ScoutLightCmd cmd{};
	cmd.enable_ctrl = 1;
	cmd.front_mode = ScoutLightCmd::LightMode::CONST_OFF;
	cmd.rear_mode = ScoutLightCmd::LightMode::CUSTOM;
	cmd.front_custom_value = 0x01;
	cmd.rear_custom_value = 0x64;
	SetLightCommand(cmd);
	printf("finish sending Lightcmd \n");
	while (1)
	{
		cmd.front_mode = ScoutLightCmd::LightMode::CONST_ON;
		cmd.rear_mode = ScoutLightCmd::LightMode::CONST_ON;
		SetLightCommand(cmd);
		boost::this_thread::sleep(boost::posix_time::seconds(1));
		cmd.front_mode = ScoutLightCmd::LightMode::CONST_OFF;
		cmd.rear_mode = ScoutLightCmd::LightMode::CONST_OFF;
		SetLightCommand(cmd);
		printf("finish sending Lightcmd \n");
		boost::this_thread::sleep(boost::posix_time::seconds(1));

	}

	//定义运动控制参数
	//定义控制指令类对象
	current_motion_cmd_.angular_velocity = 0;
	current_motion_cmd_.linear_velocity = 1;
	boost::this_thread::sleep(boost::posix_time::seconds(1));
	//本代码中所有sleep(小数)指令错误，会直接sleep(0)
	while (1)
	{


		SendMotionCmd();
		printf("sending motioncmd %d \n", i);
		//i++;
	}

	printf("finish sending motioncmd \n");

}

