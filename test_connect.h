#ifndef TEST_CONNECY_H
#define TEST_CONNECT_H

#include<connector_base.h>
#include <boost/asio.hpp>
class test_connect : public Connector
{
private:
    /* data */
public:
    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::socket* Core_socket;

    test_connect(int baud);

    // 向对端发送报文
    int  Send(const void *buf,const int buflen);


    int init(std::string Ipaddress, int port);
    void unpack_all();
    void printall();
    

    
    
    void cmd_test();
};






#endif